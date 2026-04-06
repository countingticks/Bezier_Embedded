/**
 * @brief Move List Executor implementation.
 *
 * Lifecycle:
 *   1. Pi sends N "#moves:speed;steer;time_ms;ref_x_mm;ref_y_mm;ref_heading_mrad;progress_mm;;"
 *      messages and the executor buffers the keyframes.
 *   2. Pi sends    "#moveGo:1;1;arrival_tolerance_mm;hold_final;;" and execution starts.
 *   3. Nucleo estimates current path progress from the pose and interpolates
 *      feedforward speed/steer plus reference pose on every tick.
 *   4. Encoder + IMU reconstruct the actual pose locally and apply body-frame feedback.
 *   5. Progress/telemetry is reported periodically until the end of the path
 *      is reached inside the configured arrival window.
 */

#include <brain/movelistexecutor.hpp>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
    static const uint32_t FNV_OFFSET_BASIS = 2166136261u;
    static const uint32_t FNV_PRIME = 16777619u;

    static const float PI_FLOAT = 3.14159265358979323846f;
    static const float START_HEADING_RAD = PI_FLOAT * 0.5f;
    static const float MIN_CONTROLLER_DT_S = 0.001f;
    static const float MIN_PROGRESS_STEP_MM = 1e-3f;
    static const float STARTUP_MIN_EFFECTIVE_SPEED_MM_S = 40.0f;
    static const float STARTUP_RELEASE_SPEED_MM_S = 8.0f;
    static const float STARTUP_RELEASE_PROGRESS_MM = 6.0f;
    static const uint32_t STARTUP_ASSIST_MAX_MS = 1200U;

    static const float SPEED_KP = 2.0f;            // mm/s per mm
    static const float SPEED_KI = 0.5f;            // mm/s per (mm*s)
    static const float SPEED_KD = 0.05f;            // mm/s per (mm/s)
    // static const float SPEED_KP = 0.42f;            // mm/s per mm
    // static const float SPEED_KI = 0.15f;            // mm/s per (mm*s)
    // static const float SPEED_KD = 0.02f;            // mm/s per (mm/s)
    static const float SPEED_INTEGRAL_LIMIT = 1800.0f;
    static const float SPEED_FEEDBACK_LIMIT = 180.0f;

    static const float STEER_LATERAL_KP = 5.0f;    // deci-deg per mm
    static const float STEER_LATERAL_KI = 0.85f;
    static const float STEER_LATERAL_KD = 0.1f;    // deci-deg per (mm/s)
    // static const float STEER_LATERAL_KP = 0.24f;    // deci-deg per mm
    // static const float STEER_LATERAL_KI = 0.0f;
    // static const float STEER_LATERAL_KD = 0.08f;    // deci-deg per (mm/s)
    static const float STEER_LATERAL_INTEGRAL_LIMIT = 250.0f;
    static const float STEER_HEADING_K = 420.0f;    // deci-deg per rad
    static const float STEER_FEEDBACK_LIMIT = 110.0f;

    static int16_t roundToInt16(double value)
    {
        return static_cast<int16_t>(value >= 0.0 ? value + 0.5 : value - 0.5);
    }

    static int32_t roundToInt32(double value)
    {
        return static_cast<int32_t>(value >= 0.0 ? value + 0.5 : value - 0.5);
    }
}

namespace brain
{
    CMovelistexecutor::CMovelistexecutor(
            std::chrono::milliseconds   f_period,
            UnbufferedSerial&           f_serialPort,
            drivers::ISteeringCommand&  f_steeringControl,
            drivers::ISpeedingCommand&  f_speedingControl,
            periodics::CEncoder&        f_encoder,
            periodics::CImu&            f_imu
        )
        : utils::CTask(f_period)
        , m_serialPort(f_serialPort)
        , m_steeringControl(f_steeringControl)
        , m_speedingControl(f_speedingControl)
        , m_encoder(f_encoder)
        , m_imu(f_imu)
        , m_moveCount(0)
        , m_hasReferenceTrajectory(false)
        , m_feedbackEnabled(true)
        , m_holdFinalUntilArrival(false)
        , m_arrivalToleranceMm(0U)
        , m_executing(false)
        , m_currentMove(0)
        , m_elapsedMs(0)
        , m_period(static_cast<uint32_t>(f_period.count()))
        , m_lastProgressReportMs(0)
        , m_uploadChecksum(FNV_OFFSET_BASIS)
        , m_poseReady(false)
        , m_headingInitialized(false)
        , m_initialYawDeg(0.0f)
        , m_poseXmm(0.0f)
        , m_poseYmm(0.0f)
        , m_poseHeadingRad(START_HEADING_RAD)
        , m_referenceXmm(0.0f)
        , m_referenceYmm(0.0f)
        , m_referenceHeadingRad(START_HEADING_RAD)
        , m_referenceProgressMm(0.0f)
        , m_estimatedProgressMm(0.0f)
        , m_relativeXErrorMm(0.0f)
        , m_relativeYErrorMm(0.0f)
        , m_headingErrorRad(0.0f)
        , m_speedPid(SPEED_KP, SPEED_KI, SPEED_KD, SPEED_INTEGRAL_LIMIT, SPEED_FEEDBACK_LIMIT)
        , m_lateralPid(STEER_LATERAL_KP, STEER_LATERAL_KI, STEER_LATERAL_KD, STEER_LATERAL_INTEGRAL_LIMIT, STEER_FEEDBACK_LIMIT)
        , m_headingGain(STEER_HEADING_K)
        , m_lastCommandedSpeed(0)
        , m_lastCommandedSteer(0)
    {
    }

    CMovelistexecutor::~CMovelistexecutor()
    {
    };

    void CMovelistexecutor::_run()
    {
        if (!m_executing || m_moveCount == 0)
        {
            return;
        }

        const uint32_t totalTimeMs = m_moves[m_moveCount - 1].time_ms;
        const bool waitForArrival = m_hasReferenceTrajectory && m_holdFinalUntilArrival;

        if (m_elapsedMs <= (0xFFFFFFFFu - m_period))
        {
            m_elapsedMs += m_period;
        }

        applyInterpolatedCommand(m_elapsedMs);

        const bool reachedGoal = waitForArrival && hasReachedGoal();
        if ((m_elapsedMs - m_lastProgressReportMs) >= MOVE_PROGRESS_INTERVAL_MS ||
            (!waitForArrival && m_elapsedMs >= totalTimeMs) ||
            reachedGoal)
        {
            sendProgressUpdate();
            m_lastProgressReportMs = m_elapsedMs;
        }

        if ((!waitForArrival && m_elapsedMs >= totalTimeMs) || reachedGoal)
        {
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "@moveDone:%lu;;\r\n", static_cast<unsigned long>(m_elapsedMs));
            m_serialPort.write(buffer, strlen(buffer));

            m_speedingControl.setSpeed(0);
            m_steeringControl.setAngle(0);
            resetExecutionState(true);
        }
    }

    void CMovelistexecutor::serialCallbackMovesCommand(char const * message, char * response)
    {
        if (uint8_globalsV_value_of_kl != 30)
        {
            sprintf(response, "kl 30 is required!!");
            return;
        }

        if (m_executing)
        {
            sprintf(response, "busy");
            return;
        }

        int speed = 0;
        int steer = 0;
        unsigned long timeMs = 0UL;
        long refX = 0L;
        long refY = 0L;
        int refHeading = 0;
        unsigned long progressMm = 0UL;

        const int parsed = sscanf(
            message,
            "%d;%d;%lu;%ld;%ld;%d;%lu",
            &speed,
            &steer,
            &timeMs,
            &refX,
            &refY,
            &refHeading,
            &progressMm
        );
        const bool hasReferenceFrame = (parsed == 6 || parsed == 7);

        if (parsed != 3 && parsed != 6 && parsed != 7)
        {
            sprintf(response, "syntax error");
            return;
        }

        if (m_moveCount >= MAX_MOVES)
        {
            sprintf(response, "full");
            return;
        }

        if (m_moveCount == 0)
        {
            if (timeMs != 0UL)
            {
                sprintf(response, "first time must be zero");
                return;
            }
            m_uploadChecksum = FNV_OFFSET_BASIS;
            m_hasReferenceTrajectory = hasReferenceFrame;
        }
        else
        {
            if (timeMs <= m_moves[m_moveCount - 1].time_ms)
            {
                sprintf(response, "time order");
                return;
            }

            if (hasReferenceFrame != m_hasReferenceTrajectory)
            {
                sprintf(response, "format mismatch");
                return;
            }
        }

        m_moves[m_moveCount].speed = static_cast<int16_t>(speed);
        m_moves[m_moveCount].steer = static_cast<int16_t>(steer);
        m_moves[m_moveCount].time_ms = static_cast<uint32_t>(timeMs);
        m_moves[m_moveCount].ref_x_mm = hasReferenceFrame ? static_cast<int32_t>(refX) : 0;
        m_moves[m_moveCount].ref_y_mm = hasReferenceFrame ? static_cast<int32_t>(refY) : 0;
        m_moves[m_moveCount].ref_heading_mrad = hasReferenceFrame ? static_cast<int16_t>(refHeading) : 0;
        if (hasReferenceFrame)
        {
            if (m_moveCount == 0)
            {
                m_moves[m_moveCount].progress_mm = (parsed == 7) ? static_cast<uint32_t>(progressMm) : 0U;
                if (m_moves[m_moveCount].progress_mm != 0U)
                {
                    sprintf(response, "first progress must be zero");
                    return;
                }
            }
            else if (parsed == 7)
            {
                m_moves[m_moveCount].progress_mm = static_cast<uint32_t>(progressMm);
            }
            else
            {
                const float dx = static_cast<float>(m_moves[m_moveCount].ref_x_mm - m_moves[m_moveCount - 1].ref_x_mm);
                const float dy = static_cast<float>(m_moves[m_moveCount].ref_y_mm - m_moves[m_moveCount - 1].ref_y_mm);
                m_moves[m_moveCount].progress_mm = m_moves[m_moveCount - 1].progress_mm +
                                                   static_cast<uint32_t>(sqrtf(dx * dx + dy * dy) + 0.5f);
            }

            if (m_moveCount > 0 && m_moves[m_moveCount].progress_mm < m_moves[m_moveCount - 1].progress_mm)
            {
                sprintf(response, "progress order");
                return;
            }
        }
        else
        {
            m_moves[m_moveCount].progress_mm = 0U;
        }

        m_uploadChecksum = fnv1aMix(m_uploadChecksum, static_cast<uint16_t>(m_moves[m_moveCount].speed), 2);
        m_uploadChecksum = fnv1aMix(m_uploadChecksum, static_cast<uint16_t>(m_moves[m_moveCount].steer), 2);
        m_uploadChecksum = fnv1aMix(m_uploadChecksum, m_moves[m_moveCount].time_ms, 4);
        if (m_hasReferenceTrajectory)
        {
            m_uploadChecksum = fnv1aMix(m_uploadChecksum, static_cast<uint32_t>(m_moves[m_moveCount].ref_x_mm), 4);
            m_uploadChecksum = fnv1aMix(m_uploadChecksum, static_cast<uint32_t>(m_moves[m_moveCount].ref_y_mm), 4);
            m_uploadChecksum = fnv1aMix(m_uploadChecksum, static_cast<uint16_t>(m_moves[m_moveCount].ref_heading_mrad), 2);
            m_uploadChecksum = fnv1aMix(m_uploadChecksum, m_moves[m_moveCount].progress_mm, 4);
        }
        m_moveCount++;

        sprintf(response, "%d", m_moveCount);
    }

    void CMovelistexecutor::serialCallbackMoveGoCommand(char const * message, char * response)
    {
        if (uint8_globalsV_value_of_kl != 30)
        {
            sprintf(response, "kl 30 is required!!");
            return;
        }

        if (m_moveCount == 0)
        {
            sprintf(response, "no moves");
            return;
        }

        int activate = 1;
        int feedback = 1;
        int arrivalToleranceMm = 0;
        int holdFinal = 0;
        const int parsed = sscanf(message, "%d;%d;%d;%d", &activate, &feedback, &arrivalToleranceMm, &holdFinal);
        if (parsed != 1 && parsed != 2 && parsed != 3 && parsed != 4)
        {
            sprintf(response, "syntax error");
            return;
        }

        m_feedbackEnabled = (feedback >= 1);
        m_arrivalToleranceMm = (parsed >= 3 && arrivalToleranceMm > 0) ?
            static_cast<uint32_t>(arrivalToleranceMm) :
            0U;
        m_holdFinalUntilArrival = m_hasReferenceTrajectory &&
                                  ((parsed >= 4 && holdFinal >= 1) || (parsed == 3 && arrivalToleranceMm > 0));

        resetPoseEstimate();
        m_currentMove = 0;
        m_elapsedMs = 0;
        m_lastProgressReportMs = 0;
        m_executing = true;

        applyInterpolatedCommand(0);

        snprintf(
            response,
            96,
            "ready;%u;%lu;%lu",
            static_cast<unsigned>(m_moveCount),
            static_cast<unsigned long>(m_moves[m_moveCount - 1].time_ms),
            static_cast<unsigned long>(m_uploadChecksum)
        );

        char buffer[96];
        snprintf(
            buffer,
            sizeof(buffer),
            "@moveStarted:%u;%lu;%lu;;\r\n",
            static_cast<unsigned>(m_moveCount),
            0UL,
            static_cast<unsigned long>(m_moves[m_moveCount - 1].time_ms)
        );
        m_serialPort.write(buffer, strlen(buffer));
    }

    void CMovelistexecutor::serialCallbackMoveStopCommand(char const * message, char * response)
    {
        (void)message;
        const uint32_t elapsedMs = m_elapsedMs;
        m_speedingControl.setSpeed(0);
        m_steeringControl.setAngle(0);
        resetExecutionState(true);
        snprintf(response, 48, "stopped;%lu", static_cast<unsigned long>(elapsedMs));
    }

    void CMovelistexecutor::applyInterpolatedCommand(uint32_t elapsed_ms)
    {
        if (m_moveCount == 0)
        {
            return;
        }

        float speedFeedforward = static_cast<float>(m_moves[m_moveCount - 1].speed);
        float steerFeedforward = static_cast<float>(m_moves[m_moveCount - 1].steer);
        float referenceXmm = static_cast<float>(m_moves[m_moveCount - 1].ref_x_mm);
        float referenceYmm = static_cast<float>(m_moves[m_moveCount - 1].ref_y_mm);
        float referenceHeadingRad = static_cast<float>(m_moves[m_moveCount - 1].ref_heading_mrad) / 1000.0f;

        interpolateCommandByTime(
            elapsed_ms,
            speedFeedforward,
            steerFeedforward,
            referenceXmm,
            referenceYmm,
            referenceHeadingRad
        );

        const float timeFeedforwardSpeed = speedFeedforward;
        m_referenceXmm = referenceXmm;
        m_referenceYmm = referenceYmm;
        m_referenceHeadingRad = referenceHeadingRad;

        float speedCommand = speedFeedforward;
        float steerCommand = steerFeedforward;

        if (m_hasReferenceTrajectory)
        {
            const float dt_s = clampFloat(static_cast<float>(m_period) / 1000.0f, MIN_CONTROLLER_DT_S, 1.0f);
            updatePoseEstimate(dt_s);

            if (m_poseReady)
            {
                const float finalProgressMm = static_cast<float>(m_moves[m_moveCount - 1].progress_mm);
                float estimatedProgressMm = estimateProgressAlongTrajectory();
                if (estimatedProgressMm < m_estimatedProgressMm)
                {
                    estimatedProgressMm = m_estimatedProgressMm;
                }
                if (estimatedProgressMm > finalProgressMm)
                {
                    estimatedProgressMm = finalProgressMm;
                }
                m_estimatedProgressMm = estimatedProgressMm;

                interpolateCommandByProgress(
                    m_estimatedProgressMm,
                    speedFeedforward,
                    steerFeedforward,
                    referenceXmm,
                    referenceYmm,
                    referenceHeadingRad
                );
                m_referenceXmm = referenceXmm;
                m_referenceYmm = referenceYmm;
                m_referenceHeadingRad = referenceHeadingRad;
                m_referenceProgressMm = m_estimatedProgressMm;
                speedCommand = speedFeedforward;
                steerCommand = steerFeedforward;

                const float dx = m_referenceXmm - m_poseXmm;
                const float dy = m_referenceYmm - m_poseYmm;
                const float cosHeading = cosf(m_poseHeadingRad);
                const float sinHeading = sinf(m_poseHeadingRad);

                m_relativeXErrorMm = dx * cosHeading + dy * sinHeading;
                m_relativeYErrorMm = -dx * sinHeading + dy * cosHeading;
                m_headingErrorRad = wrapAngle(m_referenceHeadingRad - m_poseHeadingRad);

                if (m_feedbackEnabled)
                {
                    const float motionSign = (speedFeedforward < -0.5f) ? -1.0f : 1.0f;
                    const float speedFeedback = clampFloat(
                        m_speedPid.update(m_relativeXErrorMm, dt_s),
                        -SPEED_FEEDBACK_LIMIT,
                        SPEED_FEEDBACK_LIMIT
                    );
                    const float lateralFeedback = clampFloat(
                        m_lateralPid.update(m_relativeYErrorMm, dt_s),
                        -STEER_FEEDBACK_LIMIT,
                        STEER_FEEDBACK_LIMIT
                    );
                    const float headingFeedback = clampFloat(
                        m_headingGain * m_headingErrorRad,
                        -STEER_FEEDBACK_LIMIT,
                        STEER_FEEDBACK_LIMIT
                    );

                    speedCommand += speedFeedback;
                    steerCommand -= motionSign * (lateralFeedback + headingFeedback);
                }
            }
        }

        const float measuredSpeedMmPerSec = m_encoder.getLinearSpeed();
        const bool startupAssistActive =
            (elapsed_ms <= STARTUP_ASSIST_MAX_MS) &&
            (fabsf(measuredSpeedMmPerSec) < STARTUP_RELEASE_SPEED_MM_S) &&
            (m_estimatedProgressMm < STARTUP_RELEASE_PROGRESS_MM);
        if (startupAssistActive)
        {
            float launchSpeed = timeFeedforwardSpeed;
            if (fabsf(launchSpeed) < STARTUP_MIN_EFFECTIVE_SPEED_MM_S)
            {
                const float launchDirection =
                    (launchSpeed < -0.5f) ? -1.0f :
                    ((launchSpeed > 0.5f) ? 1.0f :
                    ((m_moves[m_moveCount - 1].speed < 0) ? -1.0f : 1.0f));
                launchSpeed = launchDirection * STARTUP_MIN_EFFECTIVE_SPEED_MM_S;
            }

            if (fabsf(speedCommand) < fabsf(launchSpeed))
            {
                speedCommand = launchSpeed;
            }
        }

        speedCommand = clampFloat(
            speedCommand,
            static_cast<float>(m_speedingControl.get_lower_limit()),
            static_cast<float>(m_speedingControl.get_upper_limit())
        );
        steerCommand = clampFloat(
            steerCommand,
            static_cast<float>(m_steeringControl.get_lower_limit()),
            static_cast<float>(m_steeringControl.get_upper_limit())
        );

        m_lastCommandedSpeed = roundToInt16(speedCommand);
        m_lastCommandedSteer = roundToInt16(steerCommand);
        m_speedingControl.setSpeed(m_lastCommandedSpeed);
        m_steeringControl.setAngle(m_lastCommandedSteer);
    }

    void CMovelistexecutor::interpolateCommandByTime(
        uint32_t elapsed_ms,
        float& speedFeedforward,
        float& steerFeedforward,
        float& referenceXmm,
        float& referenceYmm,
        float& referenceHeadingRad
    )
    {
        if (m_moveCount <= 1)
        {
            m_currentMove = 0;
            return;
        }

        uint16_t segmentIndex = 0;
        while (segmentIndex + 1 < m_moveCount && elapsed_ms >= m_moves[segmentIndex + 1].time_ms)
        {
            segmentIndex++;
        }
        m_currentMove = segmentIndex;

        if (segmentIndex + 1 >= m_moveCount)
        {
            return;
        }

        const MoveSegment& startFrame = m_moves[segmentIndex];
        const MoveSegment& endFrame = m_moves[segmentIndex + 1];
        if (endFrame.time_ms <= startFrame.time_ms)
        {
            return;
        }

        float alpha = static_cast<float>(elapsed_ms - startFrame.time_ms) /
                      static_cast<float>(endFrame.time_ms - startFrame.time_ms);
        alpha = clampFloat(alpha, 0.0f, 1.0f);

        speedFeedforward = static_cast<float>(startFrame.speed) +
                           (static_cast<float>(endFrame.speed - startFrame.speed) * alpha);
        steerFeedforward = static_cast<float>(startFrame.steer) +
                           (static_cast<float>(endFrame.steer - startFrame.steer) * alpha);

        if (m_hasReferenceTrajectory)
        {
            referenceXmm = static_cast<float>(startFrame.ref_x_mm) +
                           (static_cast<float>(endFrame.ref_x_mm - startFrame.ref_x_mm) * alpha);
            referenceYmm = static_cast<float>(startFrame.ref_y_mm) +
                           (static_cast<float>(endFrame.ref_y_mm - startFrame.ref_y_mm) * alpha);
            referenceHeadingRad = interpolateAngle(
                static_cast<float>(startFrame.ref_heading_mrad) / 1000.0f,
                static_cast<float>(endFrame.ref_heading_mrad) / 1000.0f,
                alpha
            );
            m_referenceProgressMm = static_cast<float>(startFrame.progress_mm) +
                                    (static_cast<float>(endFrame.progress_mm - startFrame.progress_mm) * alpha);
        }
    }

    void CMovelistexecutor::interpolateCommandByProgress(
        float progress_mm,
        float& speedFeedforward,
        float& steerFeedforward,
        float& referenceXmm,
        float& referenceYmm,
        float& referenceHeadingRad
    )
    {
        if (m_moveCount <= 1)
        {
            m_currentMove = 0;
            m_referenceProgressMm = static_cast<float>(m_moves[0].progress_mm);
            return;
        }

        uint16_t segmentIndex = 0;
        while (segmentIndex + 1 < m_moveCount &&
               progress_mm > static_cast<float>(m_moves[segmentIndex + 1].progress_mm))
        {
            segmentIndex++;
        }
        m_currentMove = segmentIndex;

        if (segmentIndex + 1 >= m_moveCount)
        {
            m_referenceProgressMm = static_cast<float>(m_moves[m_moveCount - 1].progress_mm);
            return;
        }

        const MoveSegment& startFrame = m_moves[segmentIndex];
        const MoveSegment& endFrame = m_moves[segmentIndex + 1];
        const float startProgressMm = static_cast<float>(startFrame.progress_mm);
        const float endProgressMm = static_cast<float>(endFrame.progress_mm);

        float alpha = 0.0f;
        if ((endProgressMm - startProgressMm) > MIN_PROGRESS_STEP_MM)
        {
            alpha = (progress_mm - startProgressMm) / (endProgressMm - startProgressMm);
        }
        alpha = clampFloat(alpha, 0.0f, 1.0f);

        speedFeedforward = static_cast<float>(startFrame.speed) +
                           (static_cast<float>(endFrame.speed - startFrame.speed) * alpha);
        steerFeedforward = static_cast<float>(startFrame.steer) +
                           (static_cast<float>(endFrame.steer - startFrame.steer) * alpha);
        referenceXmm = static_cast<float>(startFrame.ref_x_mm) +
                       (static_cast<float>(endFrame.ref_x_mm - startFrame.ref_x_mm) * alpha);
        referenceYmm = static_cast<float>(startFrame.ref_y_mm) +
                       (static_cast<float>(endFrame.ref_y_mm - startFrame.ref_y_mm) * alpha);
        referenceHeadingRad = interpolateAngle(
            static_cast<float>(startFrame.ref_heading_mrad) / 1000.0f,
            static_cast<float>(endFrame.ref_heading_mrad) / 1000.0f,
            alpha
        );
        m_referenceProgressMm = startProgressMm + ((endProgressMm - startProgressMm) * alpha);
    }

    float CMovelistexecutor::estimateProgressAlongTrajectory() const
    {
        if (!m_hasReferenceTrajectory || m_moveCount == 0)
        {
            return 0.0f;
        }

        if (m_moveCount == 1)
        {
            return static_cast<float>(m_moves[0].progress_mm);
        }

        float bestDistanceSquared = 3.402823466e+38F;
        float bestProgressMm = 0.0f;

        for (uint16_t index = 0; index + 1 < m_moveCount; index++)
        {
            const MoveSegment& startFrame = m_moves[index];
            const MoveSegment& endFrame = m_moves[index + 1];

            const float startX = static_cast<float>(startFrame.ref_x_mm);
            const float startY = static_cast<float>(startFrame.ref_y_mm);
            const float deltaX = static_cast<float>(endFrame.ref_x_mm - startFrame.ref_x_mm);
            const float deltaY = static_cast<float>(endFrame.ref_y_mm - startFrame.ref_y_mm);
            const float segmentLengthSquared = (deltaX * deltaX) + (deltaY * deltaY);

            float alpha = 0.0f;
            if (segmentLengthSquared > MIN_PROGRESS_STEP_MM)
            {
                alpha = ((m_poseXmm - startX) * deltaX + (m_poseYmm - startY) * deltaY) / segmentLengthSquared;
                alpha = clampFloat(alpha, 0.0f, 1.0f);
            }

            const float projectedX = startX + (deltaX * alpha);
            const float projectedY = startY + (deltaY * alpha);
            const float errorX = m_poseXmm - projectedX;
            const float errorY = m_poseYmm - projectedY;
            const float distanceSquared = (errorX * errorX) + (errorY * errorY);

            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                bestProgressMm = static_cast<float>(startFrame.progress_mm) +
                                 (static_cast<float>(endFrame.progress_mm - startFrame.progress_mm) * alpha);
            }
        }

        return bestProgressMm;
    }

    bool CMovelistexecutor::hasReachedGoal() const
    {
        if (!m_hasReferenceTrajectory || !m_poseReady || m_moveCount == 0 || m_arrivalToleranceMm == 0U)
        {
            return false;
        }

        const MoveSegment& finalFrame = m_moves[m_moveCount - 1];
        const float toleranceMm = static_cast<float>(m_arrivalToleranceMm);
        const float remainingProgressMm = static_cast<float>(finalFrame.progress_mm) - m_estimatedProgressMm;
        const float goalDx = static_cast<float>(finalFrame.ref_x_mm) - m_poseXmm;
        const float goalDy = static_cast<float>(finalFrame.ref_y_mm) - m_poseYmm;

        return remainingProgressMm <= toleranceMm &&
               ((goalDx * goalDx) + (goalDy * goalDy)) <= (toleranceMm * toleranceMm);
    }

    void CMovelistexecutor::updatePoseEstimate(float dt_s)
    {
        const float startHeadingRad =
            (m_hasReferenceTrajectory && m_moveCount > 0) ?
            static_cast<float>(m_moves[0].ref_heading_mrad) / 1000.0f :
            START_HEADING_RAD;

        if (!m_imu.hasValidYaw())
        {
            return;
        }

        const float yawDeg = m_imu.getYawDegrees();
        if (!m_headingInitialized)
        {
            m_initialYawDeg = yawDeg;
            m_headingInitialized = true;
            m_poseHeadingRad = startHeadingRad;
            m_poseReady = true;
            return;
        }

        const float yawDeltaRad = wrapAngle((yawDeg - m_initialYawDeg) * (PI_FLOAT / 180.0f));
        m_poseHeadingRad = wrapAngle(startHeadingRad - yawDeltaRad);
        m_poseReady = true;

        const float speedMmPerSec = m_encoder.getLinearSpeed();
        m_poseXmm += speedMmPerSec * cosf(m_poseHeadingRad) * dt_s;
        m_poseYmm += speedMmPerSec * sinf(m_poseHeadingRad) * dt_s;
    }

    void CMovelistexecutor::resetExecutionState(bool clearBuffer)
    {
        m_executing = false;
        m_currentMove = 0;
        m_elapsedMs = 0;
        m_lastProgressReportMs = 0;
        resetPoseEstimate();
        if (clearBuffer)
        {
            m_moveCount = 0;
            m_uploadChecksum = FNV_OFFSET_BASIS;
            m_hasReferenceTrajectory = false;
            m_holdFinalUntilArrival = false;
            m_arrivalToleranceMm = 0U;
        }
    }

    void CMovelistexecutor::resetPoseEstimate()
    {
        const bool hasStartReference = m_hasReferenceTrajectory && m_moveCount > 0;
        const float startXmm = hasStartReference ? static_cast<float>(m_moves[0].ref_x_mm) : 0.0f;
        const float startYmm = hasStartReference ? static_cast<float>(m_moves[0].ref_y_mm) : 0.0f;
        const float startHeadingRad = hasStartReference ?
            static_cast<float>(m_moves[0].ref_heading_mrad) / 1000.0f :
            START_HEADING_RAD;
        const float startProgressMm = hasStartReference ? static_cast<float>(m_moves[0].progress_mm) : 0.0f;

        m_poseReady = false;
        m_headingInitialized = false;
        m_initialYawDeg = 0.0f;
        m_poseXmm = startXmm;
        m_poseYmm = startYmm;
        m_poseHeadingRad = startHeadingRad;
        m_referenceXmm = startXmm;
        m_referenceYmm = startYmm;
        m_referenceHeadingRad = startHeadingRad;
        m_referenceProgressMm = startProgressMm;
        m_estimatedProgressMm = startProgressMm;
        m_relativeXErrorMm = 0.0f;
        m_relativeYErrorMm = 0.0f;
        m_headingErrorRad = 0.0f;
        m_speedPid.reset();
        m_lateralPid.reset();
        m_lastCommandedSpeed = 0;
        m_lastCommandedSteer = 0;
    }

    void CMovelistexecutor::sendProgressUpdate()
    {
        char buffer[192];

        if (m_hasReferenceTrajectory && m_poseReady)
        {
            const int32_t poseXmm = roundToInt32(m_poseXmm);
            const int32_t poseYmm = roundToInt32(m_poseYmm);
            const int16_t headingMrad = roundToInt16(m_poseHeadingRad * 1000.0f);
            const int32_t relativeXmm = roundToInt32(m_relativeXErrorMm);
            const int32_t relativeYmm = roundToInt32(m_relativeYErrorMm);
            const int16_t headingErrorMrad = roundToInt16(m_headingErrorRad * 1000.0f);
            const int16_t measuredSpeedMmS = roundToInt16(m_encoder.getLinearSpeed());

            snprintf(
                buffer,
                sizeof(buffer),
                "@moveProgress:%lu;%u;%ld;%ld;%d;%ld;%ld;%d;%d;%d;;\r\n",
                static_cast<unsigned long>(m_elapsedMs),
                static_cast<unsigned>(m_currentMove),
                static_cast<long>(poseXmm),
                static_cast<long>(poseYmm),
                static_cast<int>(headingMrad),
                static_cast<long>(relativeXmm),
                static_cast<long>(relativeYmm),
                static_cast<int>(headingErrorMrad),
                static_cast<int>(measuredSpeedMmS),
                static_cast<int>(m_lastCommandedSteer)
            );
        }
        else
        {
            snprintf(
                buffer,
                sizeof(buffer),
                "@moveProgress:%lu;%u;;\r\n",
                static_cast<unsigned long>(m_elapsedMs),
                static_cast<unsigned>(m_currentMove)
            );
        }

        m_serialPort.write(buffer, strlen(buffer));
    }

    float CMovelistexecutor::clampFloat(float value, float minValue, float maxValue)
    {
        if (value < minValue)
        {
            return minValue;
        }
        if (value > maxValue)
        {
            return maxValue;
        }
        return value;
    }

    float CMovelistexecutor::wrapAngle(float angle)
    {
        while (angle > PI_FLOAT)
        {
            angle -= 2.0f * PI_FLOAT;
        }
        while (angle < -PI_FLOAT)
        {
            angle += 2.0f * PI_FLOAT;
        }
        return angle;
    }

    float CMovelistexecutor::interpolateAngle(float startAngle, float endAngle, float alpha)
    {
        return wrapAngle(startAngle + wrapAngle(endAngle - startAngle) * alpha);
    }

    float PIDController::update(float error, float dt)
    {
        if (dt < MIN_CONTROLLER_DT_S)
        {
            dt = MIN_CONTROLLER_DT_S;
        }

        integral += error * dt;
        if (integral > integralLimit)
        {
            integral = integralLimit;
        }
        else if (integral < -integralLimit)
        {
            integral = -integralLimit;
        }

        float derivative = 0.0f;
        if (initialized)
        {
            derivative = (error - previousError) / dt;
        }
        else
        {
            initialized = true;
        }

        previousError = error;

        float output = kp * error + ki * integral + kd * derivative;
        if (output > outputLimit)
        {
            output = outputLimit;
        }
        else if (output < -outputLimit)
        {
            output = -outputLimit;
        }
        return output;
    }

    uint32_t CMovelistexecutor::fnv1aMix(uint32_t checksum, uint32_t value, uint8_t byteCount)
    {
        for (uint8_t index = 0; index < byteCount; index++)
        {
            checksum ^= static_cast<uint8_t>((value >> (index * 8)) & 0xFFu);
            checksum *= FNV_PRIME;
        }
        return checksum;
    }

}; // namespace brain
