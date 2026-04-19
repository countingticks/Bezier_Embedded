/**
 * @brief Move List Executor implementation.
 *
 * Lifecycle:
 *   1. Pi sends N "#moves:speed;steer;time_ms;ref_x_mm;ref_y_mm;ref_heading_mrad;progress_mm;;"
 *      messages and the executor buffers the keyframes.
 *   2. Pi sends    "#moveGo:1;1;arrival_tolerance_mm;hold_final;;" and execution starts.
 *   3. Nucleo estimates current path progress from the pose and interpolates
 *      planner feedforward plus reference pose on every tick.
 *   4. A fixed-size Level 2 MPC adds speed and steer corrections on top of
 *      the planner feedforward every 50 ms while the executor keeps the
 *      existing startup assist, stop, hold, and telemetry flows.
 */

#include <brain/movelistexecutor.hpp>
#include <brain/globalsv.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
    static const uint32_t FNV_OFFSET_BASIS = 2166136261u;
    static const uint32_t FNV_PRIME = 16777619u;

    static const float PI_FLOAT = 3.14159265358979323846f;
    static const float START_HEADING_RAD = PI_FLOAT * 0.5f;
    static const float MIN_PROGRESS_STEP_MM = 1e-3f;
    static const float STARTUP_MIN_EFFECTIVE_SPEED_MM_S = 40.0f;
    static const float STARTUP_RELEASE_SPEED_MM_S = 8.0f;
    static const float STARTUP_RELEASE_PROGRESS_MM = 6.0f;
    static const uint32_t STARTUP_ASSIST_MAX_MS = 1200U;
    static const float TRAJECTORY_RELOCALIZE_DISTANCE_MM = 120.0f;
    static const float PROGRESS_NOMINAL_LEAD_LIMIT_MM = 40.0f;
    static const float PROJECTION_TRUST_DISTANCE_MM = 40.0f;
    static const float PROJECTION_PROGRESS_ALIGNMENT_TOLERANCE_MM = 60.0f;
    static const float PROJECTION_RESEED_PROGRESS_GAP_MM = 20.0f;
    static const float PROJECTION_REJOIN_SPEED_HOLD_GAP_MM = 25.0f;
    static const uint16_t CHECKPOINT_FORWARD_WINDOW_SAMPLES = 5U;
    static const uint16_t CHECKPOINT_BACKTRACK_WINDOW_SAMPLES = CHECKPOINT_FORWARD_WINDOW_SAMPLES;
    static const float CHECKPOINT_PROMOTION_TOLERANCE_MM = 5.0f;
    static const float MAX_PROGRESS_ADVANCE_PER_CYCLE_MM = 60.0f;
    static const uint32_t MPC_SAMPLE_PERIOD_MS = 50U;
    static const uint16_t INVALID_DIRECTION_SEGMENT = 0xFFFFu;
    static const int16_t DIRECTION_SPEED_EPS_MM_S = 5;
    static const float SEGMENT_ADVANCE_TOLERANCE_MM = 5.0f;
    static const float SPEED_CORRECTION_HEADROOM_MPS = 0.10f;
    static const float TRACKING_RECOVERY_SPEED_CAP_MPS = 0.08f;
    static const float TRACKING_RECOVERY_HEADING_START_RAD = 0.34906585f;
    static const float TRACKING_RECOVERY_HEADING_FULL_RAD = 1.30899694f;
    static const float TRACKING_RECOVERY_LATERAL_START_M = 0.05f;
    static const float TRACKING_RECOVERY_LATERAL_FULL_M = 0.18f;
    static const float TRACKING_RECOVERY_DISTANCE_START_M = 0.10f;
    static const float TRACKING_RECOVERY_DISTANCE_FULL_M = 0.30f;
    static const float TRACKING_RECOVERY_PROGRESS_GAP_START_M = 0.03f;
    static const float TRACKING_RECOVERY_PROGRESS_GAP_FULL_M = 0.12f;
    static const float TRACKING_RECOVERY_SPEED_RATE_MAX_MPS2 = 6.0f;
    static const float REVERSE_TRAVEL_DIAGNOSTIC_THRESHOLD_MM = 0.5f;
    static const float YAW_JUMP_DIAGNOSTIC_THRESHOLD_RAD = 0.52359878f;
    static const float MIN_TRAVEL_DIRECTION_CONFIDENCE_MM_S = 20.0f;
    static const float MAX_OPPOSITE_TRAVEL_JITTER_MM = 15.0f;

    static int16_t roundToInt16(double value)
    {
        return static_cast<int16_t>(value >= 0.0 ? value + 0.5 : value - 0.5);
    }

    static int32_t roundToInt32(double value)
    {
        return static_cast<int32_t>(value >= 0.0 ? value + 0.5 : value - 0.5);
    }

    static int8_t classifySpeedSample(int16_t speedMmS)
    {
        if (speedMmS > DIRECTION_SPEED_EPS_MM_S)
        {
            return 1;
        }

        if (speedMmS < -DIRECTION_SPEED_EPS_MM_S)
        {
            return -1;
        }

        return 0;
    }

    static float rampToUnitInterval(float value, float startValue, float endValue)
    {
        if (value <= startValue)
        {
            return 0.0f;
        }

        if (value >= endValue)
        {
            return 1.0f;
        }

        return (value - startValue) / (endValue - startValue);
    }

}

namespace brain
{
    CMovelistexecutor::CMovelistexecutor(
            std::chrono::milliseconds   f_period,
            drivers::CSerialTxBroker&   f_serialBroker,
            drivers::ISteeringCommand&  f_steeringControl,
            drivers::ISpeedingCommand&  f_speedingControl,
            periodics::CEncoder&        f_encoder,
            periodics::CImu&            f_imu
        )
        : utils::CTask(f_period)
        , m_serialBroker(f_serialBroker)
        , m_steeringControl(f_steeringControl)
        , m_speedingControl(f_speedingControl)
        , m_encoder(f_encoder)
        , m_imu(f_imu)
        , m_moves()
        , m_moveCount(0)
        , m_hasReferenceTrajectory(false)
        , m_feedbackEnabled(true)
        , m_holdFinalUntilArrival(false)
        , m_arrivalToleranceMm(0U)
        , m_executing(false)
        , m_currentMove(0U)
        , m_acceptedMoveIndex(0U)
        , m_projectedMoveIndex(0U)
        , m_elapsedMs(0U)
        , m_executionStartTime()
        , m_lastRunTime()
        , m_directionSegmentCount(0U)
        , m_activeDirectionSegment(INVALID_DIRECTION_SEGMENT)
        , m_nextMpcSolveMs(0U)
        , m_lastProgressReportMs(0U)
        , m_progressReportIntervalMs(MOVE_PROGRESS_INTERVAL_DEFAULT_MS)
        , m_uploadChecksum(FNV_OFFSET_BASIS)
        , m_lastExecutorLoopDtMs(0U)
        , m_maxExecutorLoopDtMs(0U)
        , m_lastMpcSolveUs(0U)
        , m_maxMpcSolveUs(0U)
        , m_missedMpcSolveSlots(0U)
        , m_poseSnapshotSeq(0U)
        , m_reverseTravelCount(0U)
        , m_yawJumpCount(0U)
        , m_lastHorizonSeedTimeMs(0U)
        , m_poseReady(false)
        , m_headingInitialized(false)
        , m_lastYawDeg(0.0f)
        , m_lastImuYawDeg(0.0f)
        , m_accumulatedYawDeltaRad(0.0f)
        , m_poseXmm(0.0f)
        , m_poseYmm(0.0f)
        , m_poseHeadingRad(START_HEADING_RAD)
        , m_encoderTravelMm(0.0f)
        , m_lastEncoderTravelDeltaMm(0.0f)
        , m_lastTravelDeltaMm(0.0f)
        , m_reportTravelDeltaMm(0.0f)
        , m_reportedEncoderTravelMm(0.0f)
        , m_odometryProgressMm(0.0f)
        , m_referenceXmm(0.0f)
        , m_referenceYmm(0.0f)
        , m_referenceHeadingRad(START_HEADING_RAD)
        , m_referenceProgressMm(0.0f)
        , m_rawProjectedProgressMm(0.0f)
        , m_matchedProgressMm(0.0f)
        , m_lastValidProjectedProgressMm(0.0f)
        , m_relativeXErrorMm(0.0f)
        , m_relativeYErrorMm(0.0f)
        , m_headingErrorRad(0.0f)
        , m_mpcProgressErrorM(0.0f)
        , m_mpcLateralErrorM(0.0f)
        , m_mpcHeadingErrorRad(0.0f)
        , m_cachedPathCorrectionMps(0.0f)
        , m_cachedSteerCorrectionRad(0.0f)
        , m_lastNominalProgressMm(0.0f)
        , m_lastTrustedTravelSpeedMmS(0.0f)
        , m_lastReferenceSpeedMmS(0.0f)
        , m_lastReferenceSteerDeciDeg(0.0f)
        , m_lastReferenceCurvatureInvM(0.0f)
        , m_lastHorizonSeedProgressMm(0.0f)
        , m_lastHorizonReferenceSpeedMmS(0.0f)
        , m_lastHorizonReferenceSteerDeciDeg(0.0f)
        , m_lastPreClampCommandSpeedMmS(0.0f)
        , m_lastPreClampCommandSteerDeciDeg(0.0f)
        , m_lastSpeedCorrectionMps(0.0f)
        , m_lastSteerCorrectionRad(0.0f)
        , m_lastPathSpeedCommandMps(0.0f)
        , m_lastPathSpeedLimitMps(0.0f)
        , m_lastSteerCommandRad(0.0f)
        , m_lastMpcObjective(0.0f)
        , m_lastMpcMaxConstraintViolation(0.0f)
        , m_lastMpcStatus(CMpcController::SolverStatus::Disabled)
        , m_lastMpcIterations(0U)
        , m_lastReferenceSegmentIndex(INVALID_DIRECTION_SEGMENT)
        , m_lastUsedPreviousCorrection(false)
        , m_lastUsedFeedforwardOnly(true)
        , m_lastCheckpointRecoveryHold(false)
        , m_lastSpeedSaturated(false)
        , m_lastSteerSaturated(false)
        , m_lastSpeedRateLimited(false)
        , m_lastSteerRateLimited(false)
        , m_projectionValid(false)
        , m_projectionStaleCount(0U)
        , m_lastRecoverySeverityPermille(0U)
        , m_mpcController()
        , m_directionSegments()
        , m_lastCommandedSpeed(0)
        , m_lastCommandedSteer(0)
        , m_poseSnapshot()
    {
    }

    void CMovelistexecutor::_run()
    {
        if (!m_executing || m_moveCount == 0U)
        {
            return;
        }

        const Kernel::Clock::time_point currentKernelTime = Kernel::Clock::now();
        m_lastExecutorLoopDtMs = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(currentKernelTime - m_lastRunTime).count()
        );
        m_lastRunTime = currentKernelTime;
        if (m_lastExecutorLoopDtMs > m_maxExecutorLoopDtMs)
        {
            m_maxExecutorLoopDtMs = m_lastExecutorLoopDtMs;
        }

        m_elapsedMs = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(currentKernelTime - m_executionStartTime).count()
        );

        const uint32_t totalTimeMs = m_moves[m_moveCount - 1U].time_ms;
        const bool waitForArrival = m_hasReferenceTrajectory && m_holdFinalUntilArrival;

        applyInterpolatedCommand(m_elapsedMs);

        const bool reachedGoal = waitForArrival && hasReachedGoal();
        if ((m_elapsedMs - m_lastProgressReportMs) >= m_progressReportIntervalMs ||
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
            m_serialBroker.sendReliable(buffer, strlen(buffer));

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

        if (m_moveCount == 0U)
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
            if (timeMs <= m_moves[m_moveCount - 1U].time_ms)
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
            if (m_moveCount == 0U)
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
                const float dx = static_cast<float>(m_moves[m_moveCount].ref_x_mm - m_moves[m_moveCount - 1U].ref_x_mm);
                const float dy = static_cast<float>(m_moves[m_moveCount].ref_y_mm - m_moves[m_moveCount - 1U].ref_y_mm);
                m_moves[m_moveCount].progress_mm = m_moves[m_moveCount - 1U].progress_mm +
                                                   static_cast<uint32_t>(sqrtf((dx * dx) + (dy * dy)) + 0.5f);
            }

            if (m_moveCount > 0U && m_moves[m_moveCount].progress_mm < m_moves[m_moveCount - 1U].progress_mm)
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

        if (m_moveCount == 0U)
        {
            sprintf(response, "no moves");
            return;
        }

        int activate = 1;
        int feedback = 1;
        int arrivalToleranceMm = 0;
        int holdFinal = 0;
        int progressIntervalMs = static_cast<int>(MOVE_PROGRESS_INTERVAL_DEFAULT_MS);
        const int parsed = sscanf(message, "%d;%d;%d;%d;%d", &activate, &feedback, &arrivalToleranceMm, &holdFinal, &progressIntervalMs);
        if (parsed != 1 && parsed != 2 && parsed != 3 && parsed != 4 && parsed != 5)
        {
            sprintf(response, "syntax error");
            return;
        }

        m_feedbackEnabled = (feedback >= 1);
        if (parsed >= 5 && progressIntervalMs > 0)
        {
            m_progressReportIntervalMs = static_cast<uint32_t>(progressIntervalMs);
            if (m_progressReportIntervalMs < MOVE_PROGRESS_INTERVAL_MIN_MS)
            {
                m_progressReportIntervalMs = MOVE_PROGRESS_INTERVAL_MIN_MS;
            }
        }
        else
        {
            m_progressReportIntervalMs = MOVE_PROGRESS_INTERVAL_DEFAULT_MS;
        }

        m_arrivalToleranceMm = (parsed >= 3 && arrivalToleranceMm > 0) ?
            static_cast<uint32_t>(arrivalToleranceMm) :
            0U;
        m_holdFinalUntilArrival = m_hasReferenceTrajectory &&
                                  ((parsed >= 4 && holdFinal >= 1) || (parsed == 3 && arrivalToleranceMm > 0));

        rebuildDirectionSegments();
        resetPoseEstimate();
        m_currentMove = 0U;
        m_elapsedMs = 0U;
        m_lastProgressReportMs = 0U;
        m_executionStartTime = Kernel::Clock::now();
        m_lastRunTime = m_executionStartTime;
        m_executing = true;

        applyInterpolatedCommand(0U);

        snprintf(
            response,
            96,
            "ready;%u;%lu;%lu",
            static_cast<unsigned>(m_moveCount),
            static_cast<unsigned long>(m_moves[m_moveCount - 1U].time_ms),
            static_cast<unsigned long>(m_uploadChecksum)
        );

        char buffer[96];
        snprintf(
            buffer,
            sizeof(buffer),
            "@moveStarted:%u;%lu;%lu;;\r\n",
            static_cast<unsigned>(m_moveCount),
            0UL,
            static_cast<unsigned long>(m_moves[m_moveCount - 1U].time_ms)
        );
        m_serialBroker.sendReliable(buffer, strlen(buffer));
    }

    void CMovelistexecutor::serialCallbackMoveStopCommand(char const * message, char * response)
    {
        (void)message;
        const uint32_t elapsedMs = m_executing ?
            static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(Kernel::Clock::now() - m_executionStartTime).count()
            ) :
            m_elapsedMs;
        m_speedingControl.setSpeed(0);
        m_steeringControl.setAngle(0);
        resetExecutionState(true);
        snprintf(response, 48, "stopped;%lu", static_cast<unsigned long>(elapsedMs));
    }

    void CMovelistexecutor::applyInterpolatedCommand(uint32_t elapsed_ms)
    {
        if (m_moveCount == 0U)
        {
            return;
        }

        bool mpcSlotDue = false;
        uint32_t mpcSlotsDue = 0U;
        if (elapsed_ms >= m_nextMpcSolveMs)
        {
            mpcSlotDue = true;
            mpcSlotsDue = 1U + ((elapsed_ms - m_nextMpcSolveMs) / MPC_SAMPLE_PERIOD_MS);
            if (mpcSlotsDue > 1U)
            {
                m_missedMpcSolveSlots += (mpcSlotsDue - 1U);
            }
            m_nextMpcSolveMs += (mpcSlotsDue * MPC_SAMPLE_PERIOD_MS);
        }

        float timeSpeedFeedforward = static_cast<float>(m_moves[m_moveCount - 1U].speed);
        float timeSteerFeedforward = static_cast<float>(m_moves[m_moveCount - 1U].steer);
        float timeReferenceXmm = static_cast<float>(m_moves[m_moveCount - 1U].ref_x_mm);
        float timeReferenceYmm = static_cast<float>(m_moves[m_moveCount - 1U].ref_y_mm);
        float timeReferenceHeadingRad = static_cast<float>(m_moves[m_moveCount - 1U].ref_heading_mrad) / 1000.0f;
        float timeReferenceProgressMm = static_cast<float>(m_moves[m_moveCount - 1U].progress_mm);

        interpolateCommandByTime(
            elapsed_ms,
            timeSpeedFeedforward,
            timeSteerFeedforward,
            timeReferenceXmm,
            timeReferenceYmm,
            timeReferenceHeadingRad,
            timeReferenceProgressMm
        );

        const float launchFeedforwardSpeed = timeSpeedFeedforward;
        float matchedSpeedFeedforward = timeSpeedFeedforward;
        float matchedSteerFeedforward = timeSteerFeedforward;
        float speedCommand = timeSpeedFeedforward;
        float steerCommand = timeSteerFeedforward;
        float telemetryReferenceSpeedMmS = timeSpeedFeedforward;
        float telemetryReferenceSteerDeciDeg = timeSteerFeedforward;
        bool dwellSegmentActive = false;
        uint16_t directionSegmentIndex = INVALID_DIRECTION_SEGMENT;
        m_referenceXmm = timeReferenceXmm;
        m_referenceYmm = timeReferenceYmm;
        m_referenceHeadingRad = timeReferenceHeadingRad;
        m_referenceProgressMm = timeReferenceProgressMm;
        m_lastNominalProgressMm = timeReferenceProgressMm;
        m_mpcProgressErrorM = 0.0f;
        m_mpcLateralErrorM = 0.0f;
        m_mpcHeadingErrorRad = 0.0f;

        if (m_hasReferenceTrajectory)
        {
            updatePoseEstimate();

            if (m_poseReady)
            {
                float projectedDistanceSquared = 3.402823466e+38F;
                bool projectionValid = false;
                uint16_t projectedMoveIndex = m_acceptedMoveIndex;
                if (ensureActiveDirectionSegment(elapsed_ms, directionSegmentIndex))
                {
                    const DirectionSegment& activeSegment = m_directionSegments[directionSegmentIndex];
                    const float segmentStartProgressMm = getSegmentStartProgressMm(directionSegmentIndex);
                    const float segmentLimitProgressMm = getSegmentLimitProgressMm(directionSegmentIndex);

                    if (activeSegment.direction != 0)
                    {
                        const float directedTravelMm =
                            m_lastTravelDeltaMm * static_cast<float>(activeSegment.direction);
                        if (directedTravelMm > 0.0f)
                        {
                            m_odometryProgressMm += directedTravelMm;
                        }
                        else if ((directedTravelMm < -REVERSE_TRAVEL_DIAGNOSTIC_THRESHOLD_MM) &&
                                 (m_reverseTravelCount < 0xFFFFFFFFU))
                        {
                            m_reverseTravelCount++;
                        }
                    }

                    const float projectedProgressMm = estimateProgressAlongTrajectory(
                        directionSegmentIndex,
                        projectedDistanceSquared,
                        projectionValid,
                        projectedMoveIndex
                    );
                    const float projectedDistanceMm = sqrtf(projectedDistanceSquared);

                    m_odometryProgressMm = clampFloat(
                        m_odometryProgressMm,
                        segmentStartProgressMm,
                        segmentLimitProgressMm
                    );
                    const bool projectionTrusted =
                        projectionValid &&
                        (projectedDistanceMm <= PROJECTION_TRUST_DISTANCE_MM);
                    if (projectionValid)
                    {
                        m_projectionValid = true;
                        const float clampedProjectedProgressMm = clampFloat(
                            projectedProgressMm,
                            segmentStartProgressMm,
                            segmentLimitProgressMm
                        );
                        const float previousProjectedProgressMm = clampFloat(
                            m_rawProjectedProgressMm,
                            segmentStartProgressMm,
                            segmentLimitProgressMm
                        );
                        // Limit how quickly the geometric projection can move in either
                        // direction so transient checkpoint mistakes do not create large
                        // reference jumps in one control tick. Path progress itself
                        // stays monotonic so the exported closest point never walks
                        // backward along the reference.
                        const float projectedProgressLowerBoundMm = clampFloat(
                            previousProjectedProgressMm,
                            segmentStartProgressMm,
                            segmentLimitProgressMm
                        );
                        const float projectedProgressUpperBoundMm = clampFloat(
                            previousProjectedProgressMm + MAX_PROGRESS_ADVANCE_PER_CYCLE_MM,
                            segmentStartProgressMm,
                            segmentLimitProgressMm
                        );
                        m_lastValidProjectedProgressMm = clampFloat(
                            clampedProjectedProgressMm,
                            projectedProgressLowerBoundMm,
                            projectedProgressUpperBoundMm
                        );
                        m_rawProjectedProgressMm = m_lastValidProjectedProgressMm;
                        m_projectedMoveIndex = getMoveIndexForProgress(
                            directionSegmentIndex,
                            m_lastValidProjectedProgressMm
                        );
                    }
                    else
                    {
                        m_projectionValid = false;
                        m_projectedMoveIndex = m_acceptedMoveIndex;
                        m_rawProjectedProgressMm = clampFloat(
                            m_lastValidProjectedProgressMm,
                            segmentStartProgressMm,
                            segmentLimitProgressMm
                        );
                    }
                    m_matchedProgressMm = computeMatchedProgressMm(
                        directionSegmentIndex,
                        timeReferenceProgressMm,
                        m_rawProjectedProgressMm,
                        projectedDistanceMm,
                        projectionValid
                    );

                    if (projectionTrusted)
                    {
                        m_projectionStaleCount = 0U;
                    }
                    else if (m_projectionStaleCount < 0xFFFFU)
                    {
                        m_projectionStaleCount++;
                    }
                }

                float matchedReferenceXmm = timeReferenceXmm;
                float matchedReferenceYmm = timeReferenceYmm;
                float matchedReferenceHeadingRad = timeReferenceHeadingRad;
                float matchedReferenceProgressMm = timeReferenceProgressMm;

                interpolateCommandByProgress(
                    m_matchedProgressMm,
                    matchedSpeedFeedforward,
                    matchedSteerFeedforward,
                    matchedReferenceXmm,
                    matchedReferenceYmm,
                    matchedReferenceHeadingRad,
                    matchedReferenceProgressMm
                );
                m_referenceXmm = matchedReferenceXmm;
                m_referenceYmm = matchedReferenceYmm;
                m_referenceHeadingRad = matchedReferenceHeadingRad;
                m_referenceProgressMm = matchedReferenceProgressMm;
                speedCommand = matchedSpeedFeedforward;
                steerCommand = matchedSteerFeedforward;
                telemetryReferenceSpeedMmS = matchedSpeedFeedforward;
                telemetryReferenceSteerDeciDeg = matchedSteerFeedforward;

                const int8_t trackingDirection =
                    (directionSegmentIndex != INVALID_DIRECTION_SEGMENT) ?
                    m_directionSegments[directionSegmentIndex].direction :
                    ((matchedSpeedFeedforward < -0.5f) ? -1 : 1);
                const float poseTrackingHeadingRad =
                    (trackingDirection < 0) ?
                    wrapAngle(m_poseHeadingRad + PI_FLOAT) :
                    m_poseHeadingRad;
                const float referenceTrackingHeadingRad =
                    (trackingDirection < 0) ?
                    wrapAngle(m_referenceHeadingRad + PI_FLOAT) :
                    m_referenceHeadingRad;
                const float dx = m_referenceXmm - m_poseXmm;
                const float dy = m_referenceYmm - m_poseYmm;
                const float cosHeading = cosf(poseTrackingHeadingRad);
                const float sinHeading = sinf(poseTrackingHeadingRad);

                m_relativeXErrorMm = (dx * cosHeading) + (dy * sinHeading);
                m_relativeYErrorMm = (-dx * sinHeading) + (dy * cosHeading);
                m_headingErrorRad = wrapAngle(referenceTrackingHeadingRad - poseTrackingHeadingRad);
            }
        }

        refreshPoseSnapshot();

        if (directionSegmentIndex == INVALID_DIRECTION_SEGMENT)
        {
            (void)ensureActiveDirectionSegment(elapsed_ms, directionSegmentIndex);
        }

        m_lastReferenceSegmentIndex = directionSegmentIndex;
        m_lastReferenceCurvatureInvM = 0.0f;
        m_lastHorizonSeedTimeMs = elapsed_ms;
        m_lastHorizonSeedProgressMm = m_referenceProgressMm;
        m_lastHorizonReferenceSpeedMmS = matchedSpeedFeedforward;
        m_lastHorizonReferenceSteerDeciDeg = matchedSteerFeedforward;
        m_lastPathSpeedLimitMps = fabsf(millimetersToMeters(matchedSpeedFeedforward));
        m_lastCheckpointRecoveryHold = false;
        m_lastRecoverySeverityPermille = 0U;

        if (directionSegmentIndex != INVALID_DIRECTION_SEGMENT)
        {
            const DirectionSegment& activeSegment = m_directionSegments[directionSegmentIndex];
            dwellSegmentActive = (activeSegment.direction == 0);
            if (activeSegment.direction != 0)
            {
                m_lastReferenceCurvatureInvM =
                    (-static_cast<float>(activeSegment.direction) * tanf(deciDegreesToRad(matchedSteerFeedforward))) /
                    CMpcController::c_wheelbaseM;
            }

            if (dwellSegmentActive)
            {
                speedCommand = 0.0f;
                steerCommand = 0.0f;
                telemetryReferenceSpeedMmS = 0.0f;
                telemetryReferenceSteerDeciDeg = 0.0f;
                m_cachedPathCorrectionMps = 0.0f;
                m_cachedSteerCorrectionRad = 0.0f;
                m_lastMpcStatus = CMpcController::SolverStatus::FeedforwardOnly;
                m_lastMpcIterations = 0U;
                m_lastUsedPreviousCorrection = false;
                m_lastUsedFeedforwardOnly = true;
                m_lastMpcObjective = 0.0f;
                m_lastMpcMaxConstraintViolation = 0.0f;
                m_lastSpeedRateLimited = false;
                m_lastSteerRateLimited = false;
            }
            else if (m_hasReferenceTrajectory && m_poseReady && m_feedbackEnabled)
            {
                const int8_t direction = activeSegment.direction;
                const float vRefPathMps = fabsf(millimetersToMeters(matchedSpeedFeedforward));
                const float deltaFeedforwardRad = deciDegreesToRad(matchedSteerFeedforward);
                const float thetaReferenceRad =
                    (direction < 0) ?
                    wrapAngle(m_referenceHeadingRad + PI_FLOAT) :
                    m_referenceHeadingRad;
                const float poseTravelHeadingRad =
                    (direction < 0) ?
                    wrapAngle(m_poseHeadingRad + PI_FLOAT) :
                    m_poseHeadingRad;

                const float poseDeltaXM = millimetersToMeters(m_poseXmm - m_referenceXmm);
                const float poseDeltaYM = millimetersToMeters(m_poseYmm - m_referenceYmm);
                const float cosTheta = cosf(thetaReferenceRad);
                const float sinTheta = sinf(thetaReferenceRad);

                m_mpcProgressErrorM = millimetersToMeters(m_referenceProgressMm - timeReferenceProgressMm);
                m_mpcLateralErrorM = (-sinTheta * poseDeltaXM) + (cosTheta * poseDeltaYM);
                m_mpcHeadingErrorRad = wrapAngle(poseTravelHeadingRad - thetaReferenceRad);
                const CMpcController::Limits limits = makeMpcLimits(direction, vRefPathMps);

                if (mpcSlotDue)
                {
                    CMpcController::HorizonSample horizon[CMpcController::c_horizonLength];
                    const float horizonSeedProgressMm = computeHorizonSeedProgressMm(
                        directionSegmentIndex,
                        timeReferenceProgressMm
                    );
                    fillMpcHorizon(
                        elapsed_ms,
                        horizonSeedProgressMm,
                        directionSegmentIndex,
                        matchedSpeedFeedforward,
                        matchedSteerFeedforward,
                        horizon
                    );

                    CMpcController::Input mpcInput;
                    memset(&mpcInput, 0, sizeof(mpcInput));
                    mpcInput.feedback_enabled = true;
                    mpcInput.dynamic_segment = true;
                    mpcInput.direction = direction;
                    mpcInput.pose.valid = true;
                    mpcInput.pose.x_m = millimetersToMeters(m_poseXmm);
                    mpcInput.pose.y_m = millimetersToMeters(m_poseYmm);
                    mpcInput.pose.psi_rad = poseTravelHeadingRad;
                    mpcInput.current.matched_progress_m = millimetersToMeters(m_referenceProgressMm);
                    mpcInput.current.nominal_progress_m = millimetersToMeters(timeReferenceProgressMm);
                    mpcInput.current.x_m = millimetersToMeters(m_referenceXmm);
                    mpcInput.current.y_m = millimetersToMeters(m_referenceYmm);
                    mpcInput.current.psi_rad = m_referenceHeadingRad;
                    mpcInput.current.theta_rad = thetaReferenceRad;
                    mpcInput.current.v_body_mps = millimetersToMeters(matchedSpeedFeedforward);
                    mpcInput.current.delta_ff_rad = deciDegreesToRad(matchedSteerFeedforward);
                    memcpy(mpcInput.horizon, horizon, sizeof(horizon));
                    mpcInput.limits = limits;
                    mpcInput.prev_path_speed_cmd_mps = m_lastPathSpeedCommandMps;
                    mpcInput.prev_steer_cmd_rad = m_lastSteerCommandRad;

                    Timer solveTimer;
                    solveTimer.start();
                    const CMpcController::Output mpcOutput = m_mpcController.solve(mpcInput);
                    solveTimer.stop();
                    m_lastMpcSolveUs = static_cast<uint32_t>(
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            solveTimer.elapsed_time()
                        ).count()
                    );
                    if (m_lastMpcSolveUs > m_maxMpcSolveUs)
                    {
                        m_maxMpcSolveUs = m_lastMpcSolveUs;
                    }
                    m_cachedPathCorrectionMps = mpcOutput.v_corr_mps;
                    m_cachedSteerCorrectionRad = mpcOutput.delta_corr_rad;
                    m_lastMpcStatus = mpcOutput.status;
                    m_lastMpcIterations = mpcOutput.iterations;
                    m_lastUsedPreviousCorrection = mpcOutput.used_previous_correction;
                    m_lastUsedFeedforwardOnly = mpcOutput.used_feedforward_only;
                    m_mpcProgressErrorM = mpcOutput.e_s_m;
                    m_mpcLateralErrorM = mpcOutput.e_y_m;
                    m_mpcHeadingErrorRad = mpcOutput.e_psi_rad;
                    m_lastMpcObjective = mpcOutput.objective;
                    m_lastMpcMaxConstraintViolation = mpcOutput.max_constraint_violation;
                    m_lastSpeedRateLimited = mpcOutput.speed_rate_limited;
                    m_lastSteerRateLimited = mpcOutput.steer_rate_limited;
                }

                const float pathSpeedCommandMps = clampFloat(
                    vRefPathMps + m_cachedPathCorrectionMps,
                    limits.path_speed_min_mps,
                    limits.path_speed_max_mps
                );
                const float steerCommandRad = clampFloat(
                    deltaFeedforwardRad + m_cachedSteerCorrectionRad,
                    limits.steer_min_rad,
                    limits.steer_max_rad
                );

                speedCommand = static_cast<float>(direction) * (pathSpeedCommandMps * 1000.0f);
                steerCommand = steerCommandRad * (1800.0f / PI_FLOAT);
            }
            else
            {
                m_lastMpcStatus = m_feedbackEnabled ?
                    CMpcController::SolverStatus::FeedforwardOnly :
                    CMpcController::SolverStatus::Disabled;
                m_lastMpcIterations = 0U;
                m_lastUsedPreviousCorrection = false;
                m_lastUsedFeedforwardOnly = true;
                m_lastMpcObjective = 0.0f;
                m_lastMpcMaxConstraintViolation = 0.0f;
                m_lastSpeedRateLimited = false;
                m_lastSteerRateLimited = false;
            }
        }

        if (dwellSegmentActive)
        {
            speedCommand = 0.0f;
            steerCommand = 0.0f;
        }

        const float measuredSpeedMmPerSec = m_encoder.getLinearSpeed();
        const bool startupAssistActive =
            !dwellSegmentActive &&
            (elapsed_ms <= STARTUP_ASSIST_MAX_MS) &&
            (fabsf(measuredSpeedMmPerSec) < STARTUP_RELEASE_SPEED_MM_S) &&
            (m_matchedProgressMm < STARTUP_RELEASE_PROGRESS_MM);

        if (startupAssistActive)
        {
            float launchSpeed = launchFeedforwardSpeed;
            if (fabsf(launchSpeed) < STARTUP_MIN_EFFECTIVE_SPEED_MM_S)
            {
                const float fallbackLaunchDirection =
                    ((directionSegmentIndex != INVALID_DIRECTION_SEGMENT) &&
                     (m_directionSegments[directionSegmentIndex].direction != 0)) ?
                    static_cast<float>(m_directionSegments[directionSegmentIndex].direction) :
                    ((m_moves[m_moveCount - 1U].speed < 0) ? -1.0f : 1.0f);
                const float launchDirection =
                    (launchSpeed < -0.5f) ? -1.0f :
                    ((launchSpeed > 0.5f) ? 1.0f :
                    fallbackLaunchDirection);
                launchSpeed = launchDirection * STARTUP_MIN_EFFECTIVE_SPEED_MM_S;
            }

            if (fabsf(speedCommand) < fabsf(launchSpeed))
            {
                speedCommand = launchSpeed;
            }
        }

        m_lastPreClampCommandSpeedMmS = speedCommand;
        m_lastPreClampCommandSteerDeciDeg = steerCommand;
        const float clampedSpeedCommand = clampFloat(
            speedCommand,
            static_cast<float>(m_speedingControl.get_lower_limit()),
            static_cast<float>(m_speedingControl.get_upper_limit())
        );
        const float clampedSteerCommand = clampFloat(
            steerCommand,
            static_cast<float>(m_steeringControl.get_lower_limit()),
            static_cast<float>(m_steeringControl.get_upper_limit())
        );
        m_lastSpeedSaturated = (fabsf(clampedSpeedCommand - m_lastPreClampCommandSpeedMmS) > 0.5f);
        m_lastSteerSaturated = (fabsf(clampedSteerCommand - m_lastPreClampCommandSteerDeciDeg) > 0.5f);
        speedCommand = clampedSpeedCommand;
        steerCommand = clampedSteerCommand;

        m_lastReferenceSpeedMmS = telemetryReferenceSpeedMmS;
        m_lastReferenceSteerDeciDeg = telemetryReferenceSteerDeciDeg;
        m_lastCommandedSpeed = roundToInt16(speedCommand);
        m_lastCommandedSteer = roundToInt16(steerCommand);
        m_lastSpeedCorrectionMps = millimetersToMeters(
            static_cast<float>(m_lastCommandedSpeed) - m_lastReferenceSpeedMmS
        );
        m_lastSteerCorrectionRad = deciDegreesToRad(
            static_cast<float>(m_lastCommandedSteer) - m_lastReferenceSteerDeciDeg
        );
        m_lastPathSpeedCommandMps = fabsf(millimetersToMeters(static_cast<float>(m_lastCommandedSpeed)));
        m_lastSteerCommandRad = deciDegreesToRad(static_cast<float>(m_lastCommandedSteer));

        m_speedingControl.setSpeed(m_lastCommandedSpeed);
        m_steeringControl.setAngle(m_lastCommandedSteer);
    }

    void CMovelistexecutor::interpolateCommandByTime(
        uint32_t elapsed_ms,
        float& speedFeedforward,
        float& steerFeedforward,
        float& referenceXmm,
        float& referenceYmm,
        float& referenceHeadingRad,
        float& referenceProgressMm
    )
    {
        if (m_moveCount <= 1U)
        {
            m_currentMove = 0U;
            referenceProgressMm = (m_moveCount == 1U) ? static_cast<float>(m_moves[0].progress_mm) : 0.0f;
            return;
        }

        uint16_t segmentIndex = m_currentMove;
        if (segmentIndex >= m_moveCount)
        {
            segmentIndex = 0U;
        }

        while (segmentIndex > 0U && elapsed_ms < m_moves[segmentIndex].time_ms)
        {
            segmentIndex--;
        }

        while ((segmentIndex + 1U) < m_moveCount && elapsed_ms >= m_moves[segmentIndex + 1U].time_ms)
        {
            segmentIndex++;
        }
        m_currentMove = segmentIndex;

        if ((segmentIndex + 1U) >= m_moveCount)
        {
            referenceProgressMm = static_cast<float>(m_moves[m_moveCount - 1U].progress_mm);
            return;
        }

        const MoveSegment& startFrame = m_moves[segmentIndex];
        const MoveSegment& endFrame = m_moves[segmentIndex + 1U];
        if (endFrame.time_ms <= startFrame.time_ms)
        {
            referenceProgressMm = static_cast<float>(startFrame.progress_mm);
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
            referenceProgressMm = static_cast<float>(startFrame.progress_mm) +
                                  (static_cast<float>(endFrame.progress_mm - startFrame.progress_mm) * alpha);
        }
        else
        {
            referenceProgressMm = 0.0f;
        }
    }

    void CMovelistexecutor::interpolateCommandByProgress(
        float progress_mm,
        float& speedFeedforward,
        float& steerFeedforward,
        float& referenceXmm,
        float& referenceYmm,
        float& referenceHeadingRad,
        float& referenceProgressMm
    )
    {
        if (m_moveCount <= 1U)
        {
            m_currentMove = 0U;
            referenceProgressMm = (m_moveCount == 1U) ? static_cast<float>(m_moves[0].progress_mm) : 0.0f;
            return;
        }

        uint16_t segmentIndex = m_currentMove;
        if (segmentIndex >= m_moveCount)
        {
            segmentIndex = 0U;
        }

        while (segmentIndex > 0U &&
               progress_mm < static_cast<float>(m_moves[segmentIndex].progress_mm))
        {
            segmentIndex--;
        }

        while ((segmentIndex + 1U) < m_moveCount &&
               progress_mm > static_cast<float>(m_moves[segmentIndex + 1U].progress_mm))
        {
            segmentIndex++;
        }
        m_currentMove = segmentIndex;

        if ((segmentIndex + 1U) >= m_moveCount)
        {
            referenceProgressMm = static_cast<float>(m_moves[m_moveCount - 1U].progress_mm);
            return;
        }

        const MoveSegment& startFrame = m_moves[segmentIndex];
        const MoveSegment& endFrame = m_moves[segmentIndex + 1U];
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
        referenceProgressMm = startProgressMm + ((endProgressMm - startProgressMm) * alpha);
    }

    float CMovelistexecutor::estimateProgressAlongTrajectory(
        uint16_t directionSegmentIndex,
        float& bestDistanceSquared,
        bool& projectionValid,
        uint16_t& projectedMoveIndex
    )
    {
        if (!m_hasReferenceTrajectory || m_moveCount == 0U)
        {
            bestDistanceSquared = 0.0f;
            projectionValid = false;
            projectedMoveIndex = 0U;
            return 0.0f;
        }

        if (m_moveCount == 1U)
        {
            bestDistanceSquared = 0.0f;
            projectionValid = true;
            projectedMoveIndex = 0U;
            return static_cast<float>(m_moves[0].progress_mm);
        }

        bestDistanceSquared = 3.402823466e+38F;
        uint16_t searchStart = 0U;
        uint16_t searchEnd = static_cast<uint16_t>(m_moveCount - 2U);

        if (directionSegmentIndex < m_directionSegmentCount)
        {
            const DirectionSegment& activeSegment = m_directionSegments[directionSegmentIndex];
            searchStart = activeSegment.start_index;
            if (activeSegment.end_index > activeSegment.start_index)
            {
                searchEnd = static_cast<uint16_t>(activeSegment.end_index - 1U);
            }
            else
            {
                searchEnd = activeSegment.start_index;
            }

            if (searchEnd >= (m_moveCount - 1U))
            {
                searchEnd = static_cast<uint16_t>(m_moveCount - 2U);
            }
        }

        if (searchStart > searchEnd)
        {
            searchStart = searchEnd;
        }

        if (m_acceptedMoveIndex < searchStart)
        {
            m_acceptedMoveIndex = searchStart;
        }
        if (m_acceptedMoveIndex > searchEnd)
        {
            m_acceptedMoveIndex = searchEnd;
        }
        const float acceptedProgressMm = static_cast<float>(m_moves[m_acceptedMoveIndex].progress_mm);
        float anchorProgressMm = acceptedProgressMm;
        if (m_matchedProgressMm > (anchorProgressMm + PROJECTION_RESEED_PROGRESS_GAP_MM))
        {
            anchorProgressMm = m_matchedProgressMm;
        }
        if (m_odometryProgressMm > (anchorProgressMm + PROJECTION_RESEED_PROGRESS_GAP_MM))
        {
            anchorProgressMm = m_odometryProgressMm;
        }
        if (m_lastNominalProgressMm > (anchorProgressMm + PROJECTION_RESEED_PROGRESS_GAP_MM))
        {
            anchorProgressMm = m_lastNominalProgressMm;
        }

        uint16_t searchAnchor = m_acceptedMoveIndex;
        if (anchorProgressMm > (acceptedProgressMm + PROJECTION_RESEED_PROGRESS_GAP_MM))
        {
            const uint16_t anchorMoveIndex = getMoveIndexForProgress(
                directionSegmentIndex,
                anchorProgressMm
            );
            if (anchorMoveIndex > searchAnchor)
            {
                searchAnchor = anchorMoveIndex;
            }
        }

        uint16_t windowStart = searchAnchor;
        if (windowStart > searchStart)
        {
            const uint16_t backtrackStart =
                (windowStart > CHECKPOINT_BACKTRACK_WINDOW_SAMPLES) ?
                static_cast<uint16_t>(windowStart - CHECKPOINT_BACKTRACK_WINDOW_SAMPLES) :
                0U;
            windowStart = (backtrackStart > searchStart) ? backtrackStart : searchStart;
        }
        const uint16_t windowEnd = getProjectionSearchEndMoveIndex(directionSegmentIndex, searchAnchor);
        uint16_t bestSegment = searchAnchor;
        float bestProgressMm = estimateProgressInRange(
            windowStart,
            windowEnd,
            bestDistanceSquared,
            bestSegment
        );

        const float relocalizeDistanceSquared = TRAJECTORY_RELOCALIZE_DISTANCE_MM * TRAJECTORY_RELOCALIZE_DISTANCE_MM;
        const bool projectionLaggingAnchor =
            bestProgressMm + PROJECTION_RESEED_PROGRESS_GAP_MM < anchorProgressMm;
        if (((bestDistanceSquared > relocalizeDistanceSquared) || projectionLaggingAnchor) &&
            ((windowStart > searchStart) || (windowEnd < searchEnd)))
        {
            float segmentBestDistanceSquared = 3.402823466e+38F;
            uint16_t segmentBestSegment = searchAnchor;
            const float segmentBestProgressMm = estimateProgressInRange(
                searchStart,
                searchEnd,
                segmentBestDistanceSquared,
                segmentBestSegment
            );

            if (segmentBestDistanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = segmentBestDistanceSquared;
                bestSegment = segmentBestSegment;
                bestProgressMm = segmentBestProgressMm;
            }
        }

        projectionValid = (bestDistanceSquared <= relocalizeDistanceSquared);
        projectedMoveIndex = projectionValid ? bestSegment : m_acceptedMoveIndex;

        if (!projectionValid)
        {
            const float checkpointProgressMm = static_cast<float>(m_moves[m_acceptedMoveIndex].progress_mm);
            return (m_lastValidProjectedProgressMm > checkpointProgressMm) ?
                m_lastValidProjectedProgressMm :
                checkpointProgressMm;
        }

        return bestProgressMm;
    }

    bool CMovelistexecutor::hasReachedGoal() const
    {
        if (!m_hasReferenceTrajectory || !m_poseReady || m_moveCount == 0U || m_arrivalToleranceMm == 0U)
        {
            return false;
        }

        const MoveSegment& finalFrame = m_moves[m_moveCount - 1U];
        const float toleranceMm = static_cast<float>(m_arrivalToleranceMm);
        const float remainingProgressMm = static_cast<float>(finalFrame.progress_mm) - m_matchedProgressMm;
        const float goalDx = static_cast<float>(finalFrame.ref_x_mm) - m_poseXmm;
        const float goalDy = static_cast<float>(finalFrame.ref_y_mm) - m_poseYmm;

        return remainingProgressMm <= toleranceMm &&
               ((goalDx * goalDx) + (goalDy * goalDy)) <= (toleranceMm * toleranceMm);
    }

    void CMovelistexecutor::updatePoseEstimate()
    {
        m_lastEncoderTravelDeltaMm = 0.0f;
        m_lastTravelDeltaMm = 0.0f;

        const float startHeadingRad =
            (m_hasReferenceTrajectory && m_moveCount > 0U) ?
            static_cast<float>(m_moves[0].ref_heading_mrad) / 1000.0f :
            START_HEADING_RAD;

        if (!m_imu.hasValidYaw())
        {
            return;
        }

        const float yawDeg = m_imu.getYawDegrees();
        m_lastImuYawDeg = yawDeg;
        const float previousPoseHeadingRad = m_poseHeadingRad;
        if (!m_headingInitialized)
        {
            m_lastYawDeg = yawDeg;
            m_accumulatedYawDeltaRad = 0.0f;
            m_headingInitialized = true;
            m_poseHeadingRad = startHeadingRad;
            m_poseReady = true;
            return;
        }

        const float yawStepRad = wrapAngle((yawDeg - m_lastYawDeg) * (PI_FLOAT / 180.0f));
        m_lastYawDeg = yawDeg;
        if ((fabsf(yawStepRad) > YAW_JUMP_DIAGNOSTIC_THRESHOLD_RAD) &&
            (m_yawJumpCount < 0xFFFFFFFFU))
        {
            m_yawJumpCount++;
        }
        m_accumulatedYawDeltaRad += yawStepRad;
        m_poseHeadingRad = wrapAngle(startHeadingRad - m_accumulatedYawDeltaRad);
        m_poseReady = true;

        const float encoderTravelMm = m_encoder.getTravelDistanceMm();
        const float travelDeltaMm = encoderTravelMm - m_encoderTravelMm;
        m_encoderTravelMm = encoderTravelMm;
        m_lastEncoderTravelDeltaMm = travelDeltaMm;
        const float measuredSpeedMmPerSec = m_encoder.getLinearSpeed();

        float filteredTravelDeltaMm = travelDeltaMm;
        int8_t expectedTravelDirection = 0;
        if (m_activeDirectionSegment < m_directionSegmentCount)
        {
            const int8_t activeDirection = m_directionSegments[m_activeDirectionSegment].direction;
            if (activeDirection != 0)
            {
                expectedTravelDirection = activeDirection;
            }
        }

        if (expectedTravelDirection == 0)
        {
            if (fabsf(measuredSpeedMmPerSec) >= MIN_TRAVEL_DIRECTION_CONFIDENCE_MM_S)
            {
                expectedTravelDirection = (measuredSpeedMmPerSec > 0.0f) ? 1 : -1;
            }
            else if (std::abs(m_lastCommandedSpeed) >= static_cast<int16_t>(MIN_TRAVEL_DIRECTION_CONFIDENCE_MM_S))
            {
                expectedTravelDirection = (m_lastCommandedSpeed > 0) ? 1 : -1;
            }
        }
        if ((expectedTravelDirection != 0) &&
            ((measuredSpeedMmPerSec * static_cast<float>(expectedTravelDirection)) > MIN_TRAVEL_DIRECTION_CONFIDENCE_MM_S))
        {
            m_lastTrustedTravelSpeedMmS = measuredSpeedMmPerSec;
        }

        if ((expectedTravelDirection != 0) &&
            ((filteredTravelDeltaMm * static_cast<float>(expectedTravelDirection)) < 0.0f))
        {
            if (fabsf(filteredTravelDeltaMm) <= MAX_OPPOSITE_TRAVEL_JITTER_MM)
            {
                filteredTravelDeltaMm = 0.0f;
            }
            else
            {
                float fallbackTravelDeltaMm = 0.0f;
                if ((m_lastExecutorLoopDtMs > 0U) &&
                    ((m_lastTrustedTravelSpeedMmS * static_cast<float>(expectedTravelDirection)) >
                     MIN_TRAVEL_DIRECTION_CONFIDENCE_MM_S))
                {
                    fallbackTravelDeltaMm =
                        fabsf(m_lastTrustedTravelSpeedMmS) *
                        (static_cast<float>(m_lastExecutorLoopDtMs) / 1000.0f);
                }

                if (fallbackTravelDeltaMm > 0.0f)
                {
                    filteredTravelDeltaMm = static_cast<float>(expectedTravelDirection) * fallbackTravelDeltaMm;
                }
                else
                {
                    filteredTravelDeltaMm = 0.0f;
                }
            }
        }

        const float localizationDeltaMm = filteredTravelDeltaMm;
        m_lastTravelDeltaMm = localizationDeltaMm;
        m_reportTravelDeltaMm += localizationDeltaMm;
        m_reportedEncoderTravelMm += fabsf(localizationDeltaMm);

        const float translationHeadingRad = interpolateAngle(previousPoseHeadingRad, m_poseHeadingRad, 0.5f);
        m_poseXmm += localizationDeltaMm * cosf(translationHeadingRad);
        m_poseYmm += localizationDeltaMm * sinf(translationHeadingRad);
    }

    void CMovelistexecutor::refreshPoseSnapshot()
    {
        if (m_poseSnapshotSeq < 0xFFFFFFFFU)
        {
            m_poseSnapshotSeq++;
        }

        m_poseSnapshot.x_mm = m_poseXmm;
        m_poseSnapshot.y_mm = m_poseYmm;
        m_poseSnapshot.heading_rad = m_poseHeadingRad;
        m_poseSnapshot.matched_progress_mm = m_matchedProgressMm;
        m_poseSnapshot.raw_projected_progress_mm = m_rawProjectedProgressMm;
        m_poseSnapshot.odometry_progress_mm = m_odometryProgressMm;
        m_poseSnapshot.seq = m_poseSnapshotSeq;
        m_poseSnapshot.executing = m_executing;
        m_poseSnapshot.valid = m_executing &&
                               m_hasReferenceTrajectory &&
                               (m_moveCount > 0U) &&
                               m_poseReady;
    }

    void CMovelistexecutor::resetExecutionState(bool clearBuffer)
    {
        m_executing = false;
        m_currentMove = 0U;
        m_acceptedMoveIndex = 0U;
        m_projectedMoveIndex = 0U;
        m_elapsedMs = 0U;
        m_executionStartTime = Kernel::Clock::time_point();
        m_lastRunTime = Kernel::Clock::time_point();
        m_lastProgressReportMs = 0U;
        resetPoseEstimate();

        if (clearBuffer)
        {
            m_moveCount = 0U;
            m_uploadChecksum = FNV_OFFSET_BASIS;
            m_hasReferenceTrajectory = false;
            m_holdFinalUntilArrival = false;
            m_arrivalToleranceMm = 0U;
            m_directionSegmentCount = 0U;
            m_activeDirectionSegment = INVALID_DIRECTION_SEGMENT;
        }
    }

    void CMovelistexecutor::resetPoseEstimate()
    {
        const bool hasStartReference = m_hasReferenceTrajectory && m_moveCount > 0U;
        const float startXmm = hasStartReference ? static_cast<float>(m_moves[0].ref_x_mm) : 0.0f;
        const float startYmm = hasStartReference ? static_cast<float>(m_moves[0].ref_y_mm) : 0.0f;
        const float startHeadingRad = hasStartReference ?
            static_cast<float>(m_moves[0].ref_heading_mrad) / 1000.0f :
            START_HEADING_RAD;
        const float startProgressMm = hasStartReference ? static_cast<float>(m_moves[0].progress_mm) : 0.0f;

        m_poseReady = false;
        m_headingInitialized = false;
        m_lastYawDeg = 0.0f;
        m_lastImuYawDeg = 0.0f;
        m_accumulatedYawDeltaRad = 0.0f;
        m_poseXmm = startXmm;
        m_poseYmm = startYmm;
        m_poseHeadingRad = startHeadingRad;
        m_encoder.resetTravelDistance();
        m_encoderTravelMm = 0.0f;
        m_lastEncoderTravelDeltaMm = 0.0f;
        m_lastTravelDeltaMm = 0.0f;
        m_reportTravelDeltaMm = 0.0f;
        m_reportedEncoderTravelMm = 0.0f;
        m_odometryProgressMm = startProgressMm;
        m_referenceXmm = startXmm;
        m_referenceYmm = startYmm;
        m_referenceHeadingRad = startHeadingRad;
        m_referenceProgressMm = startProgressMm;
        m_rawProjectedProgressMm = startProgressMm;
        m_matchedProgressMm = startProgressMm;
        m_lastValidProjectedProgressMm = startProgressMm;
        m_relativeXErrorMm = 0.0f;
        m_relativeYErrorMm = 0.0f;
        m_headingErrorRad = 0.0f;
        m_lastNominalProgressMm = startProgressMm;
        m_lastTrustedTravelSpeedMmS = 0.0f;
        m_lastReferenceSpeedMmS = 0.0f;
        m_lastReferenceSteerDeciDeg = 0.0f;
        m_lastCommandedSpeed = 0;
        m_lastCommandedSteer = 0;
        m_acceptedMoveIndex = 0U;
        m_projectedMoveIndex = 0U;
        m_poseSnapshotSeq = 0U;
        m_reverseTravelCount = 0U;
        m_yawJumpCount = 0U;
        m_lastHorizonSeedTimeMs = 0U;
        m_lastReferenceCurvatureInvM = 0.0f;
        m_lastHorizonSeedProgressMm = startProgressMm;
        m_lastHorizonReferenceSpeedMmS = 0.0f;
        m_lastHorizonReferenceSteerDeciDeg = 0.0f;
        m_lastPreClampCommandSpeedMmS = 0.0f;
        m_lastPreClampCommandSteerDeciDeg = 0.0f;
        resetControllerState();
        refreshPoseSnapshot();
    }

    void CMovelistexecutor::resetControllerState()
    {
        m_activeDirectionSegment = INVALID_DIRECTION_SEGMENT;
        m_nextMpcSolveMs = 0U;
        m_lastExecutorLoopDtMs = 0U;
        m_maxExecutorLoopDtMs = 0U;
        m_lastMpcSolveUs = 0U;
        m_maxMpcSolveUs = 0U;
        m_missedMpcSolveSlots = 0U;
        m_mpcProgressErrorM = 0.0f;
        m_mpcLateralErrorM = 0.0f;
        m_mpcHeadingErrorRad = 0.0f;
        m_cachedPathCorrectionMps = 0.0f;
        m_cachedSteerCorrectionRad = 0.0f;
        m_lastSpeedCorrectionMps = 0.0f;
        m_lastSteerCorrectionRad = 0.0f;
        m_lastPathSpeedCommandMps = 0.0f;
        m_lastPathSpeedLimitMps = 0.0f;
        m_lastSteerCommandRad = 0.0f;
        m_lastMpcObjective = 0.0f;
        m_lastMpcMaxConstraintViolation = 0.0f;
        m_lastMpcStatus = CMpcController::SolverStatus::Disabled;
        m_lastMpcIterations = 0U;
        m_lastReferenceSegmentIndex = INVALID_DIRECTION_SEGMENT;
        m_lastUsedPreviousCorrection = false;
        m_lastUsedFeedforwardOnly = true;
        m_lastCheckpointRecoveryHold = false;
        m_lastSpeedSaturated = false;
        m_lastSteerSaturated = false;
        m_lastSpeedRateLimited = false;
        m_lastSteerRateLimited = false;
        m_projectionValid = false;
        m_projectionStaleCount = 0U;
        m_lastRecoverySeverityPermille = 0U;
        m_mpcController.reset();
    }

    void CMovelistexecutor::sendProgressUpdate()
    {
        char buffer[1024];
        const ExecutorPoseSnapshot poseSnapshot = getPoseSnapshot();

        if (poseSnapshot.valid)
        {
            const int32_t poseXmm = roundToInt32(poseSnapshot.x_mm);
            const int32_t poseYmm = roundToInt32(poseSnapshot.y_mm);
            const int16_t headingMrad = roundToInt16(poseSnapshot.heading_rad * 1000.0f);
            const int32_t relativeXmm = roundToInt32(m_relativeXErrorMm);
            const int32_t relativeYmm = roundToInt32(m_relativeYErrorMm);
            const int16_t headingErrorMrad = roundToInt16(m_headingErrorRad * 1000.0f);
            const int16_t measuredSpeedMmS = roundToInt16(m_encoder.getLinearSpeed());
            const int32_t matchedProgressMm = roundToInt32(poseSnapshot.matched_progress_mm);
            const int32_t nominalProgressMm = roundToInt32(m_lastNominalProgressMm);
            const int32_t referenceXmm = roundToInt32(m_referenceXmm);
            const int32_t referenceYmm = roundToInt32(m_referenceYmm);
            const int16_t referenceHeadingMrad = roundToInt16(m_referenceHeadingRad * 1000.0f);
            const int16_t commandSpeedMmS = m_lastCommandedSpeed;
            const int16_t referenceSpeedMmS = roundToInt16(m_lastReferenceSpeedMmS);
            const int16_t referenceSteerDeciDeg = roundToInt16(m_lastReferenceSteerDeciDeg);
            const int16_t speedCorrectionMmS = roundToInt16(m_lastSpeedCorrectionMps * 1000.0f);
            const int16_t steerCorrectionDeciDeg = roundToInt16(
                m_lastSteerCorrectionRad * (1800.0f / PI_FLOAT)
            );
            const int32_t rawProjectedProgressMm = roundToInt32(poseSnapshot.raw_projected_progress_mm);
            const int32_t odometryProgressMm = roundToInt32(poseSnapshot.odometry_progress_mm);
            const int32_t travelDeltaMm = roundToInt32(m_reportTravelDeltaMm);
            const int32_t encoderTravelMm = roundToInt32(m_reportedEncoderTravelMm);
            const int32_t encoderDisplacementMdeg = roundToInt32(m_encoder.getTotalDisplacementDegrees() * 1000.0f);
            const int16_t imuYawMrad = roundToInt16(m_lastImuYawDeg * (PI_FLOAT * 1000.0f / 180.0f));
            const int32_t referenceCurvatureMilliInvM = roundToInt32(m_lastReferenceCurvatureInvM * 1000.0f);
            const int32_t horizonSeedProgressMm = roundToInt32(m_lastHorizonSeedProgressMm);
            const int16_t horizonReferenceSpeedMmS = roundToInt16(m_lastHorizonReferenceSpeedMmS);
            const int16_t horizonReferenceSteerDeciDeg = roundToInt16(m_lastHorizonReferenceSteerDeciDeg);
            const int16_t preClampCommandSpeedMmS = roundToInt16(m_lastPreClampCommandSpeedMmS);
            const int16_t preClampCommandSteerDeciDeg = roundToInt16(m_lastPreClampCommandSteerDeciDeg);
            const int32_t mpcObjectiveMilli = roundToInt32(m_lastMpcObjective * 1000.0f);
            const int32_t mpcMaxConstraintViolationMicro = roundToInt32(
                m_lastMpcMaxConstraintViolation * 1000000.0f
            );
            const uint32_t pathSpeedLimitMmS = static_cast<uint32_t>(roundf(m_lastPathSpeedLimitMps * 1000.0f));
            const uint32_t recoverySeverityPermille = static_cast<uint32_t>(m_lastRecoverySeverityPermille);
            const uint32_t checkpointRecoveryHold =
                m_lastCheckpointRecoveryHold ? 1U : 0U;
            const uint32_t encoderRejectedMeasurements = m_encoder.getRejectedMeasurementCount();
            const uint32_t encoderRejectedDeltaMdeg = static_cast<uint32_t>(
                roundf(m_encoder.getLastRejectedDeltaDegrees() * 1000.0f)
            );
            const uint32_t encoderRejectedLimitMdeg = static_cast<uint32_t>(
                roundf(m_encoder.getLastRejectedLimitDegrees() * 1000.0f)
            );
            const uint32_t encoderMissingMeasurementMs = m_encoder.getMissingMeasurementDurationMs();

            int length = snprintf(
                buffer,
                sizeof(buffer),
                "@moveProgress:%lu;%u;%ld;%ld;%d;%ld;%ld;%d;%d;%d;%ld;%ld;%ld;%ld;%d;%d;%d;%d;%d;%d;%u;%u;%u;%u;%ld;%ld;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu",
                static_cast<unsigned long>(m_elapsedMs),
                static_cast<unsigned>(m_currentMove),
                static_cast<long>(poseXmm),
                static_cast<long>(poseYmm),
                static_cast<int>(headingMrad),
                static_cast<long>(relativeXmm),
                static_cast<long>(relativeYmm),
                static_cast<int>(headingErrorMrad),
                static_cast<int>(measuredSpeedMmS),
                static_cast<int>(m_lastCommandedSteer),
                static_cast<long>(matchedProgressMm),
                static_cast<long>(nominalProgressMm),
                static_cast<long>(referenceXmm),
                static_cast<long>(referenceYmm),
                static_cast<int>(referenceHeadingMrad),
                static_cast<int>(commandSpeedMmS),
                static_cast<int>(referenceSpeedMmS),
                static_cast<int>(referenceSteerDeciDeg),
                static_cast<int>(speedCorrectionMmS),
                static_cast<int>(steerCorrectionDeciDeg),
                static_cast<unsigned>(m_lastMpcStatus),
                static_cast<unsigned>(m_lastMpcIterations),
                static_cast<unsigned>(m_lastUsedPreviousCorrection ? 1U : 0U),
                static_cast<unsigned>(m_lastUsedFeedforwardOnly ? 1U : 0U),
                static_cast<long>(rawProjectedProgressMm),
                static_cast<long>(odometryProgressMm),
                static_cast<unsigned long>(m_lastExecutorLoopDtMs),
                static_cast<unsigned long>(m_maxExecutorLoopDtMs),
                static_cast<unsigned long>(m_lastMpcSolveUs),
                static_cast<unsigned long>(m_maxMpcSolveUs),
                static_cast<unsigned long>(m_missedMpcSolveSlots),
                static_cast<unsigned long>(pathSpeedLimitMmS),
                static_cast<unsigned long>(recoverySeverityPermille),
                static_cast<unsigned long>(checkpointRecoveryHold),
                static_cast<unsigned long>(encoderRejectedMeasurements),
                static_cast<unsigned long>(encoderRejectedDeltaMdeg),
                static_cast<unsigned long>(encoderRejectedLimitMdeg),
                static_cast<unsigned long>(encoderMissingMeasurementMs)
            );
            if (length > 0 && static_cast<size_t>(length) < sizeof(buffer))
            {
                snprintf(
                    buffer + length,
                    sizeof(buffer) - static_cast<size_t>(length),
                    ";%ld;%ld;%ld;%d;%ld;%u;%lu;%ld;%d;%d;%d;%d;%u;%u;%u;%u;%ld;%ld;;\r\n",
                    static_cast<long>(travelDeltaMm),
                    static_cast<long>(encoderTravelMm),
                    static_cast<long>(encoderDisplacementMdeg),
                    static_cast<int>(imuYawMrad),
                    static_cast<long>(referenceCurvatureMilliInvM),
                    static_cast<unsigned>(m_lastReferenceSegmentIndex),
                    static_cast<unsigned long>(m_lastHorizonSeedTimeMs),
                    static_cast<long>(horizonSeedProgressMm),
                    static_cast<int>(horizonReferenceSpeedMmS),
                    static_cast<int>(horizonReferenceSteerDeciDeg),
                    static_cast<int>(preClampCommandSpeedMmS),
                    static_cast<int>(preClampCommandSteerDeciDeg),
                    static_cast<unsigned>(m_lastSpeedSaturated ? 1U : 0U),
                    static_cast<unsigned>(m_lastSteerSaturated ? 1U : 0U),
                    static_cast<unsigned>(m_lastSpeedRateLimited ? 1U : 0U),
                    static_cast<unsigned>(m_lastSteerRateLimited ? 1U : 0U),
                    static_cast<long>(mpcObjectiveMilli),
                    static_cast<long>(mpcMaxConstraintViolationMicro)
                );
            }
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

        m_serialBroker.publishLatest(drivers::CSerialTxBroker::TelemetryTopic::MoveProgress, buffer, strlen(buffer));
        m_reportTravelDeltaMm = 0.0f;
        m_maxExecutorLoopDtMs = m_lastExecutorLoopDtMs;
        m_maxMpcSolveUs = m_lastMpcSolveUs;
    }

    void CMovelistexecutor::rebuildDirectionSegments()
    {
        m_directionSegmentCount = 0U;
        if (m_moveCount == 0U)
        {
            return;
        }

        if (m_moveCount == 1U)
        {
            m_directionSegments[0].start_index = 0U;
            m_directionSegments[0].end_index = 0U;
            m_directionSegments[0].time_start_ms = m_moves[0].time_ms;
            m_directionSegments[0].time_end_ms = m_moves[0].time_ms;
            m_directionSegments[0].progress_start_mm = m_moves[0].progress_mm;
            m_directionSegments[0].progress_end_mm = m_moves[0].progress_mm;
            m_directionSegments[0].direction = classifyDirection(m_moves[0].speed, m_moves[0].speed);
            m_directionSegmentCount = 1U;
            return;
        }

        uint16_t segmentStartIndex = 0U;
        int8_t activeDirection = classifyDirection(m_moves[0].speed, m_moves[1].speed);

        for (uint16_t intervalIndex = 1U; intervalIndex < (m_moveCount - 1U); intervalIndex++)
        {
            const int8_t nextDirection = classifyDirection(m_moves[intervalIndex].speed, m_moves[intervalIndex + 1U].speed);
            if (nextDirection == activeDirection)
            {
                continue;
            }

            DirectionSegment& segment = m_directionSegments[m_directionSegmentCount++];
            segment.start_index = segmentStartIndex;
            segment.end_index = intervalIndex;
            segment.time_start_ms = m_moves[segmentStartIndex].time_ms;
            segment.time_end_ms = m_moves[intervalIndex].time_ms;
            segment.progress_start_mm = m_moves[segmentStartIndex].progress_mm;
            segment.progress_end_mm = m_moves[intervalIndex].progress_mm;
            segment.direction = activeDirection;

            segmentStartIndex = intervalIndex;
            activeDirection = nextDirection;
        }

        DirectionSegment& finalSegment = m_directionSegments[m_directionSegmentCount++];
        finalSegment.start_index = segmentStartIndex;
        finalSegment.end_index = static_cast<uint16_t>(m_moveCount - 1U);
        finalSegment.time_start_ms = m_moves[segmentStartIndex].time_ms;
        finalSegment.time_end_ms = m_moves[m_moveCount - 1U].time_ms;
        finalSegment.progress_start_mm = m_moves[segmentStartIndex].progress_mm;
        finalSegment.progress_end_mm = m_moves[m_moveCount - 1U].progress_mm;
        finalSegment.direction = activeDirection;
    }

    bool CMovelistexecutor::ensureActiveDirectionSegment(uint32_t elapsedMs, uint16_t& directionSegmentIndex)
    {
        if (m_directionSegmentCount == 0U)
        {
            directionSegmentIndex = INVALID_DIRECTION_SEGMENT;
            return false;
        }

        if (m_activeDirectionSegment == INVALID_DIRECTION_SEGMENT)
        {
            activateDirectionSegment(0U);
        }

        directionSegmentIndex = m_activeDirectionSegment;
        while ((directionSegmentIndex + 1U) < m_directionSegmentCount)
        {
            const DirectionSegment& activeSegment = m_directionSegments[directionSegmentIndex];

            if (activeSegment.direction == 0)
            {
                if (elapsedMs < activeSegment.time_end_ms)
                {
                    break;
                }
            }
            else
            {
                uint16_t endpointIndex = activeSegment.end_index;
                uint32_t endpointTimeMs = activeSegment.time_end_ms;
                float endpointProgressMm = static_cast<float>(activeSegment.progress_end_mm);
                (void)getDynamicSegmentEndpoint(
                    directionSegmentIndex,
                    endpointIndex,
                    endpointTimeMs,
                    endpointProgressMm
                );

                const bool progressReachedEnd =
                    m_matchedProgressMm + SEGMENT_ADVANCE_TOLERANCE_MM >= endpointProgressMm;
                const bool checkpointReachedEnd =
                    (endpointIndex == 0U) || (m_acceptedMoveIndex + 1U >= endpointIndex);

                if (!progressReachedEnd || !checkpointReachedEnd)
                {
                    break;
                }
            }

            activateDirectionSegment(static_cast<uint16_t>(directionSegmentIndex + 1U));
            directionSegmentIndex = m_activeDirectionSegment;
        }

        return directionSegmentIndex != INVALID_DIRECTION_SEGMENT;
    }

    void CMovelistexecutor::activateDirectionSegment(uint16_t directionSegmentIndex)
    {
        if (directionSegmentIndex >= m_directionSegmentCount)
        {
            directionSegmentIndex = static_cast<uint16_t>(m_directionSegmentCount - 1U);
        }

        if (m_activeDirectionSegment == directionSegmentIndex)
        {
            return;
        }

        const DirectionSegment& activeSegment = m_directionSegments[directionSegmentIndex];
        const float segmentStartProgressMm = static_cast<float>(activeSegment.progress_start_mm);
        const float segmentLimitProgressMm = getSegmentLimitProgressMm(directionSegmentIndex);

        m_activeDirectionSegment = directionSegmentIndex;
        m_odometryProgressMm = clampFloat(m_odometryProgressMm, segmentStartProgressMm, segmentLimitProgressMm);
        m_rawProjectedProgressMm = clampFloat(m_rawProjectedProgressMm, segmentStartProgressMm, segmentLimitProgressMm);
        m_matchedProgressMm = clampFloat(
            (m_matchedProgressMm < segmentStartProgressMm) ? segmentStartProgressMm : m_matchedProgressMm,
            segmentStartProgressMm,
            segmentLimitProgressMm
        );
        m_referenceProgressMm = m_matchedProgressMm;
        m_acceptedMoveIndex = getMoveIndexForProgress(directionSegmentIndex, m_matchedProgressMm);
        m_projectedMoveIndex = m_acceptedMoveIndex;
        m_lastValidProjectedProgressMm = m_matchedProgressMm;
        m_projectionValid = false;
        m_projectionStaleCount = 0U;

        m_mpcController.resetWarmStart();
        m_cachedPathCorrectionMps = 0.0f;
        m_cachedSteerCorrectionRad = 0.0f;
        m_lastMpcStatus = CMpcController::SolverStatus::Disabled;
        m_lastMpcIterations = 0U;
        m_lastUsedPreviousCorrection = false;
        m_lastUsedFeedforwardOnly = true;
    }

    float CMovelistexecutor::getSegmentStartProgressMm(uint16_t directionSegmentIndex) const
    {
        if (directionSegmentIndex >= m_directionSegmentCount)
        {
            return 0.0f;
        }

        return static_cast<float>(m_directionSegments[directionSegmentIndex].progress_start_mm);
    }

    uint16_t CMovelistexecutor::getMoveIndexForProgress(uint16_t directionSegmentIndex, float progressMm) const
    {
        if (m_moveCount <= 1U)
        {
            return 0U;
        }

        uint16_t searchStart = 0U;
        uint16_t searchEnd = static_cast<uint16_t>(m_moveCount - 2U);
        if (directionSegmentIndex < m_directionSegmentCount)
        {
            const DirectionSegment& activeSegment = m_directionSegments[directionSegmentIndex];
            searchStart = activeSegment.start_index;
            if (activeSegment.end_index > activeSegment.start_index)
            {
                searchEnd = static_cast<uint16_t>(activeSegment.end_index - 1U);
            }
            else
            {
                searchEnd = activeSegment.start_index;
            }

            if (searchEnd >= (m_moveCount - 1U))
            {
                searchEnd = static_cast<uint16_t>(m_moveCount - 2U);
            }
        }

        if (searchStart > searchEnd)
        {
            searchStart = searchEnd;
        }

        uint16_t moveIndex = searchStart;
        while ((moveIndex < searchEnd) &&
               (progressMm > static_cast<float>(m_moves[moveIndex + 1U].progress_mm)))
        {
            moveIndex++;
        }

        return moveIndex;
    }

    uint16_t CMovelistexecutor::getCheckpointWindowEndMoveIndex(uint16_t directionSegmentIndex, uint16_t acceptedMoveIndex) const
    {
        return getProjectionSearchEndMoveIndex(directionSegmentIndex, acceptedMoveIndex);
    }

    uint16_t CMovelistexecutor::getProjectionSearchEndMoveIndex(uint16_t directionSegmentIndex, uint16_t acceptedMoveIndex) const
    {
        if (m_moveCount <= 1U)
        {
            return 0U;
        }

        uint16_t segmentEndMoveIndex = static_cast<uint16_t>(m_moveCount - 2U);
        if (directionSegmentIndex < m_directionSegmentCount)
        {
            const DirectionSegment& activeSegment = m_directionSegments[directionSegmentIndex];
            if (activeSegment.end_index > activeSegment.start_index)
            {
                segmentEndMoveIndex = static_cast<uint16_t>(activeSegment.end_index - 1U);
            }
            else
            {
                segmentEndMoveIndex = activeSegment.start_index;
            }

            if (segmentEndMoveIndex >= (m_moveCount - 1U))
            {
                segmentEndMoveIndex = static_cast<uint16_t>(m_moveCount - 2U);
            }

            if (acceptedMoveIndex < activeSegment.start_index)
            {
                acceptedMoveIndex = activeSegment.start_index;
            }
            if (acceptedMoveIndex > segmentEndMoveIndex)
            {
                acceptedMoveIndex = segmentEndMoveIndex;
            }
        }

        uint32_t requestedWindowEnd =
            static_cast<uint32_t>(acceptedMoveIndex) + static_cast<uint32_t>(CHECKPOINT_FORWARD_WINDOW_SAMPLES);
        if (requestedWindowEnd > static_cast<uint32_t>(segmentEndMoveIndex))
        {
            requestedWindowEnd = static_cast<uint32_t>(segmentEndMoveIndex);
        }

        return static_cast<uint16_t>(requestedWindowEnd);
    }

    float CMovelistexecutor::getSegmentLimitProgressMm(uint16_t directionSegmentIndex) const
    {
        if (directionSegmentIndex >= m_directionSegmentCount)
        {
            return 0.0f;
        }

        uint16_t endpointIndex = m_directionSegments[directionSegmentIndex].end_index;
        uint32_t endpointTimeMs = m_directionSegments[directionSegmentIndex].time_end_ms;
        float endpointProgressMm = static_cast<float>(m_directionSegments[directionSegmentIndex].progress_end_mm);

        if (getDynamicSegmentEndpoint(directionSegmentIndex, endpointIndex, endpointTimeMs, endpointProgressMm))
        {
            return endpointProgressMm;
        }

        return static_cast<float>(m_directionSegments[directionSegmentIndex].progress_end_mm);
    }

    float CMovelistexecutor::computeMatchedProgressMm(
        uint16_t directionSegmentIndex,
        float nominalProgressMm,
        float rawProjectedProgressMm,
        float projectedDistanceMm,
        bool projectionValid
    )
    {
        const float segmentLimitProgressMm = getSegmentLimitProgressMm(directionSegmentIndex);
        if (m_moveCount == 0U)
        {
            return 0.0f;
        }
        const float previousMatchedProgressMm = m_matchedProgressMm;

        uint16_t segmentStartMoveIndex = 0U;
        if (directionSegmentIndex < m_directionSegmentCount)
        {
            segmentStartMoveIndex = m_directionSegments[directionSegmentIndex].start_index;
        }
        const float segmentStartProgressMm = static_cast<float>(m_moves[segmentStartMoveIndex].progress_mm);
        const float rawOdometryProgressMm = clampFloat(
            m_odometryProgressMm,
            segmentStartProgressMm,
            segmentLimitProgressMm
        );
        float alignedMatchedFloorMm = previousMatchedProgressMm;
        if (previousMatchedProgressMm > (rawOdometryProgressMm + PROJECTION_PROGRESS_ALIGNMENT_TOLERANCE_MM))
        {
            alignedMatchedFloorMm = rawOdometryProgressMm + PROJECTION_PROGRESS_ALIGNMENT_TOLERANCE_MM;
            const uint16_t alignedMoveIndex = getMoveIndexForProgress(
                directionSegmentIndex,
                alignedMatchedFloorMm
            );
            if (alignedMoveIndex < m_acceptedMoveIndex)
            {
                m_acceptedMoveIndex = alignedMoveIndex;
            }
        }

        const uint16_t checkpointWindowEndMoveIndex = getCheckpointWindowEndMoveIndex(
            directionSegmentIndex,
            m_acceptedMoveIndex
        );
        if (m_acceptedMoveIndex < segmentStartMoveIndex)
        {
            m_acceptedMoveIndex = segmentStartMoveIndex;
        }
        if (m_acceptedMoveIndex > checkpointWindowEndMoveIndex)
        {
            m_acceptedMoveIndex = checkpointWindowEndMoveIndex;
        }
        const float checkpointFloorMm = static_cast<float>(m_moves[m_acceptedMoveIndex].progress_mm);
        float checkpointCeilingMm = segmentLimitProgressMm;
        if ((checkpointWindowEndMoveIndex + 1U) < m_moveCount)
        {
            checkpointCeilingMm = clampFloat(
                static_cast<float>(m_moves[checkpointWindowEndMoveIndex + 1U].progress_mm),
                checkpointFloorMm,
                segmentLimitProgressMm
            );
        }
        float cycleAdvanceCeilingMm = clampFloat(
            previousMatchedProgressMm + MAX_PROGRESS_ADVANCE_PER_CYCLE_MM,
            checkpointFloorMm,
            checkpointCeilingMm
        );
        if (projectionValid &&
            (projectedDistanceMm <= TRAJECTORY_RELOCALIZE_DISTANCE_MM))
        {
            // Once the geometric projection is valid, allow matched progress to
            // rejoin it in the same cycle instead of staying artificially pinned
            // behind a stale odometry estimate.
            const float projectedCatchupCeilingMm = clampFloat(
                rawProjectedProgressMm,
                checkpointFloorMm,
                checkpointCeilingMm
            );
            if (projectedCatchupCeilingMm > cycleAdvanceCeilingMm)
            {
                cycleAdvanceCeilingMm = projectedCatchupCeilingMm;
            }
        }
        const float candidateFloorMm = clampFloat(
            alignedMatchedFloorMm,
            checkpointFloorMm,
            cycleAdvanceCeilingMm
        );

        float projectedProgressMm = clampFloat(
            rawProjectedProgressMm,
            checkpointFloorMm,
            cycleAdvanceCeilingMm
        );

        const bool projectionTrusted =
            projectionValid &&
            (projectedDistanceMm <= PROJECTION_TRUST_DISTANCE_MM);
        const float odometryProgressMm = clampFloat(
            rawOdometryProgressMm,
            candidateFloorMm,
            cycleAdvanceCeilingMm
        );
        const bool projectionLaggingOdometry =
            projectedProgressMm + PROJECTION_RESEED_PROGRESS_GAP_MM < odometryProgressMm;
        const bool projectionLeadingOdometry =
            projectedProgressMm > (odometryProgressMm + PROJECTION_PROGRESS_ALIGNMENT_TOLERANCE_MM);

        float candidateProgressMm = candidateFloorMm;
        if (projectionValid &&
            (projectedDistanceMm <= TRAJECTORY_RELOCALIZE_DISTANCE_MM))
        {
            candidateProgressMm = projectedProgressMm;
            if (projectionLeadingOdometry)
            {
                // When the closest-point projection snaps ahead during a path
                // rejoin, keep matched progress anchored near odometry until
                // the two progress estimates converge again.
                candidateProgressMm = clampFloat(
                    odometryProgressMm + PROJECTION_PROGRESS_ALIGNMENT_TOLERANCE_MM,
                    candidateFloorMm,
                    cycleAdvanceCeilingMm
                );
            }
        }
        // If the geometric match goes stale behind the car, keep the exported
        // progress moving forward with odometry instead of freezing the
        // closest/reference point in place.
        if ((!projectionTrusted || projectionLaggingOdometry) &&
            (odometryProgressMm > candidateProgressMm))
        {
            candidateProgressMm = odometryProgressMm;
        }

        const float nominalLeadCeilingMm = clampFloat(
            nominalProgressMm + PROGRESS_NOMINAL_LEAD_LIMIT_MM,
            candidateFloorMm,
            cycleAdvanceCeilingMm
        );
        if (candidateProgressMm > nominalLeadCeilingMm && nominalLeadCeilingMm > candidateFloorMm)
        {
            candidateProgressMm = nominalLeadCeilingMm;
        }

        if ((directionSegmentIndex + 1U) == m_directionSegmentCount)
        {
            const MoveSegment& finalFrame = m_moves[m_moveCount - 1U];
            const float goalToleranceMm =
                (m_arrivalToleranceMm > 0U) ?
                static_cast<float>(m_arrivalToleranceMm) :
                PROJECTION_PROGRESS_ALIGNMENT_TOLERANCE_MM;
            const float goalDx = static_cast<float>(finalFrame.ref_x_mm) - m_poseXmm;
            const float goalDy = static_cast<float>(finalFrame.ref_y_mm) - m_poseYmm;
            const float goalDistanceMm = sqrtf((goalDx * goalDx) + (goalDy * goalDy));
            const float remainingProgressFloorMm = fmaxf(0.0f, goalDistanceMm - goalToleranceMm);
            const float terminalGoalCapMm = clampFloat(
                static_cast<float>(finalFrame.progress_mm) - remainingProgressFloorMm,
                candidateFloorMm,
                cycleAdvanceCeilingMm
            );
            if (candidateProgressMm > terminalGoalCapMm)
            {
                candidateProgressMm = terminalGoalCapMm;
            }
        }

        candidateProgressMm = clampFloat(
            candidateProgressMm,
            candidateFloorMm,
            cycleAdvanceCeilingMm
        );

        // Advance the accepted checkpoint only after the continuous candidate is
        // known, so the checkpoint cursor never drags the exported reference
        // ahead of the trustworthy geometric/odometric estimate.
        uint16_t promotionWindowEndMoveIndex = getCheckpointWindowEndMoveIndex(
            directionSegmentIndex,
            m_acceptedMoveIndex
        );
        while ((m_acceptedMoveIndex < promotionWindowEndMoveIndex) &&
               (static_cast<float>(m_moves[m_acceptedMoveIndex + 1U].progress_mm) <=
                (candidateProgressMm + CHECKPOINT_PROMOTION_TOLERANCE_MM)))
        {
            m_acceptedMoveIndex++;
            promotionWindowEndMoveIndex = getCheckpointWindowEndMoveIndex(
                directionSegmentIndex,
                m_acceptedMoveIndex
            );
        }

        return candidateProgressMm;
    }

    float CMovelistexecutor::computeHorizonSeedProgressMm(uint16_t directionSegmentIndex, float nominalProgressMm) const
    {
        const float segmentStartProgressMm = getSegmentStartProgressMm(directionSegmentIndex);
        const float segmentLimitProgressMm = getSegmentLimitProgressMm(directionSegmentIndex);
        const float nominalStageProgressMm = clampFloat(
            nominalProgressMm,
            segmentStartProgressMm,
            segmentLimitProgressMm
        );

        return clampFloat(
            (m_referenceProgressMm > nominalStageProgressMm) ? m_referenceProgressMm : nominalStageProgressMm,
            m_matchedProgressMm,
            segmentLimitProgressMm
        );
    }

    void CMovelistexecutor::fillMpcHorizon(
        uint32_t elapsed_ms,
        float progressSeedMm,
        uint16_t directionSegmentIndex,
        float currentSpeedFeedforwardMmS,
        float currentSteerFeedforwardDeciDeg,
        CMpcController::HorizonSample (&horizon)[CMpcController::c_horizonLength]
    )
    {
        const DirectionSegment& activeSegment = m_directionSegments[directionSegmentIndex];
        uint16_t endpointIndex = activeSegment.end_index;
        uint32_t endpointTimeMs = activeSegment.time_end_ms;
        float endpointProgressMm = static_cast<float>(activeSegment.progress_end_mm);
        if (!getDynamicSegmentEndpoint(directionSegmentIndex, endpointIndex, endpointTimeMs, endpointProgressMm))
        {
            endpointProgressMm = static_cast<float>(activeSegment.progress_end_mm);
        }

        const uint16_t savedCurrentMove = m_currentMove;
        const float clampedSeedProgressMm = clampFloat(
            progressSeedMm,
            static_cast<float>(activeSegment.progress_start_mm),
            endpointProgressMm
        );
        const uint32_t clampedSeedTimeMs = (elapsed_ms < activeSegment.time_start_ms) ?
            activeSegment.time_start_ms :
            ((elapsed_ms > endpointTimeMs) ? endpointTimeMs : elapsed_ms);

        m_lastHorizonSeedTimeMs = clampedSeedTimeMs;
        m_lastHorizonSeedProgressMm = clampedSeedProgressMm;
        m_lastHorizonReferenceSpeedMmS = currentSpeedFeedforwardMmS;
        m_lastHorizonReferenceSteerDeciDeg = currentSteerFeedforwardDeciDeg;

        horizon[0].v_body_mps = millimetersToMeters(currentSpeedFeedforwardMmS);
        horizon[0].delta_ff_rad = deciDegreesToRad(currentSteerFeedforwardDeciDeg);

        for (size_t stageIndex = 1U; stageIndex < CMpcController::c_horizonLength; stageIndex++)
        {
            uint32_t stageElapsedMs = clampedSeedTimeMs + static_cast<uint32_t>(stageIndex) * MPC_SAMPLE_PERIOD_MS;
            if (stageElapsedMs > endpointTimeMs)
            {
                stageElapsedMs = endpointTimeMs;
            }

            float speedFeedforwardMmS = static_cast<float>(m_moves[endpointIndex].speed);
            float steerFeedforwardDeciDeg = static_cast<float>(m_moves[endpointIndex].steer);
            float referenceXmm = static_cast<float>(m_moves[endpointIndex].ref_x_mm);
            float referenceYmm = static_cast<float>(m_moves[endpointIndex].ref_y_mm);
            float referenceHeadingRad = static_cast<float>(m_moves[endpointIndex].ref_heading_mrad) / 1000.0f;
            float referenceProgressMm = endpointProgressMm;

            interpolateCommandByTime(
                stageElapsedMs,
                speedFeedforwardMmS,
                steerFeedforwardDeciDeg,
                referenceXmm,
                referenceYmm,
                referenceHeadingRad,
                referenceProgressMm
            );
            if (stageIndex == 1U)
            {
                m_lastHorizonReferenceSpeedMmS = speedFeedforwardMmS;
                m_lastHorizonReferenceSteerDeciDeg = steerFeedforwardDeciDeg;
            }

            horizon[stageIndex].v_body_mps = millimetersToMeters(speedFeedforwardMmS);
            horizon[stageIndex].delta_ff_rad = deciDegreesToRad(steerFeedforwardDeciDeg);
        }

        m_currentMove = savedCurrentMove;
    }

    CMpcController::Limits CMovelistexecutor::makeMpcLimits(int8_t direction, float referencePathSpeedMps)
    {
        CMpcController::Limits limits;
        const float actuatorForwardLimitMps = millimetersToMeters(static_cast<float>(m_speedingControl.get_upper_limit()));
        const float actuatorReverseLimitMps = millimetersToMeters(static_cast<float>(-m_speedingControl.get_lower_limit()));
        const float requestedSpeedLimitMps = CMpcController::c_defaultPathSpeedMaxMps;
        const float actuatorSpeedLimitMps =
            (direction < 0) ?
            clampFloat(requestedSpeedLimitMps, 0.0f, actuatorReverseLimitMps) :
            clampFloat(requestedSpeedLimitMps, 0.0f, actuatorForwardLimitMps);

        const float headingSeverity = rampToUnitInterval(
            fabsf(m_mpcHeadingErrorRad),
            TRACKING_RECOVERY_HEADING_START_RAD,
            TRACKING_RECOVERY_HEADING_FULL_RAD
        );
        const float lateralSeverity = rampToUnitInterval(
            fabsf(m_mpcLateralErrorM),
            TRACKING_RECOVERY_LATERAL_START_M,
            TRACKING_RECOVERY_LATERAL_FULL_M
        );
        const float poseDistanceM = millimetersToMeters(
            sqrtf(
                (m_relativeXErrorMm * m_relativeXErrorMm) +
                (m_relativeYErrorMm * m_relativeYErrorMm)
            )
        );
        const float distanceSeverity = rampToUnitInterval(
            poseDistanceM,
            TRACKING_RECOVERY_DISTANCE_START_M,
            TRACKING_RECOVERY_DISTANCE_FULL_M
        );
        const bool projectionTrusted = m_projectionValid && (m_projectionStaleCount == 0U);
        const bool odometryAligned =
            fabsf(m_matchedProgressMm - m_odometryProgressMm) <= PROJECTION_PROGRESS_ALIGNMENT_TOLERANCE_MM;
        const float progressGapM = projectionTrusted ?
            millimetersToMeters(fmaxf(0.0f, m_matchedProgressMm - m_rawProjectedProgressMm)) :
            0.0f;
        const float progressGapSeverity = rampToUnitInterval(
            progressGapM,
            TRACKING_RECOVERY_PROGRESS_GAP_START_M,
            TRACKING_RECOVERY_PROGRESS_GAP_FULL_M
        );
        // When heading/lateral/path-distance errors grow, the direct-speed MPC
        // should stop trying to "catch up" on progress with extra throttle.
        const float recoverySeverity =
            fmaxf(progressGapSeverity, fmaxf(headingSeverity, fmaxf(lateralSeverity, distanceSeverity)));
        // Hold positive speed correction until the geometric projection is
        // both valid and roughly aligned with the accepted progress cursor.
        const bool checkpointRecoveryHold =
            (!m_projectionValid) ||
            ((m_projectionStaleCount > 0U) && !odometryAligned) ||
            (projectionTrusted &&
             ((m_matchedProgressMm - m_rawProjectedProgressMm) > PROJECTION_REJOIN_SPEED_HOLD_GAP_MM));

        const float positiveCorrectionCapMps = clampFloat(
            referencePathSpeedMps + (checkpointRecoveryHold ? 0.0f : SPEED_CORRECTION_HEADROOM_MPS),
            0.0f,
            actuatorSpeedLimitMps
        );
        const float recoveryCapMps = clampFloat(
            referencePathSpeedMps,
            0.0f,
            TRACKING_RECOVERY_SPEED_CAP_MPS
        );
        const float adaptiveSpeedLimitMps =
            positiveCorrectionCapMps -
            ((positiveCorrectionCapMps - recoveryCapMps) * recoverySeverity);

        limits.path_speed_min_mps = 0.0f;
        limits.path_speed_max_mps = clampFloat(
            adaptiveSpeedLimitMps,
            limits.path_speed_min_mps,
            actuatorSpeedLimitMps
        );
        m_lastPathSpeedLimitMps = limits.path_speed_max_mps;
        m_lastCheckpointRecoveryHold = checkpointRecoveryHold;
        m_lastRecoverySeverityPermille = static_cast<uint16_t>(
            roundToInt32(clampFloat(recoverySeverity, 0.0f, 1.0f) * 1000.0f)
        );

        const float actuatorSteerMinRad = deciDegreesToRad(static_cast<float>(m_steeringControl.get_lower_limit()));
        const float actuatorSteerMaxRad = deciDegreesToRad(static_cast<float>(m_steeringControl.get_upper_limit()));
        limits.steer_min_rad = clampFloat(-CMpcController::c_defaultSteerLimitRad, actuatorSteerMinRad, 0.0f);
        limits.steer_max_rad = clampFloat(CMpcController::c_defaultSteerLimitRad, 0.0f, actuatorSteerMaxRad);
        const float speedRateSeverity = checkpointRecoveryHold ? 1.0f : recoverySeverity;
        limits.speed_rate_max_mps2 =
            CMpcController::c_defaultSpeedRateLimitMps2 +
            ((TRACKING_RECOVERY_SPEED_RATE_MAX_MPS2 - CMpcController::c_defaultSpeedRateLimitMps2) * speedRateSeverity);
        limits.steer_rate_max_rad_s = CMpcController::c_defaultSteerRateLimitRadS;
        return limits;
    }

    bool CMovelistexecutor::getDynamicSegmentEndpoint(
        uint16_t directionSegmentIndex,
        uint16_t& endpointIndex,
        uint32_t& endpointTimeMs,
        float& endpointProgressMm
    ) const
    {
        if (directionSegmentIndex >= m_directionSegmentCount)
        {
            return false;
        }

        const DirectionSegment& segment = m_directionSegments[directionSegmentIndex];
        if (segment.direction == 0)
        {
            return false;
        }

        endpointIndex = segment.end_index;
        while (endpointIndex > segment.start_index &&
               classifySpeedSample(m_moves[endpointIndex].speed) == 0)
        {
            endpointIndex--;
        }

        if (classifySpeedSample(m_moves[endpointIndex].speed) == 0)
        {
            return false;
        }

        endpointTimeMs = m_moves[endpointIndex].time_ms;
        endpointProgressMm = static_cast<float>(m_moves[endpointIndex].progress_mm);
        return true;
    }

    int8_t CMovelistexecutor::classifyDirection(int16_t startSpeed, int16_t endSpeed)
    {
        const int8_t startDirection = classifySpeedSample(startSpeed);
        const int8_t endDirection = classifySpeedSample(endSpeed);

        if (startDirection == 0 && endDirection == 0)
        {
            return 0;
        }

        if (startDirection == 0)
        {
            return endDirection;
        }

        if (endDirection == 0)
        {
            return startDirection;
        }

        if (startDirection == endDirection)
        {
            return startDirection;
        }

        const int32_t startMagnitude =
            (startSpeed < 0) ? -static_cast<int32_t>(startSpeed) : static_cast<int32_t>(startSpeed);
        const int32_t endMagnitude =
            (endSpeed < 0) ? -static_cast<int32_t>(endSpeed) : static_cast<int32_t>(endSpeed);

        if (endMagnitude > startMagnitude)
        {
            return endDirection;
        }

        return startDirection;
    }

    float CMovelistexecutor::estimateProgressInRange(
        uint16_t startIndex,
        uint16_t endIndex,
        float& bestDistanceSquared,
        uint16_t& bestSegment
    ) const
    {
        float bestProgressMm = 0.0f;

        for (uint16_t index = startIndex; index <= endIndex; index++)
        {
            const MoveSegment& startFrame = m_moves[index];
            const MoveSegment& endFrame = m_moves[index + 1U];

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
                bestSegment = index;
                bestProgressMm = static_cast<float>(startFrame.progress_mm) +
                                 (static_cast<float>(endFrame.progress_mm - startFrame.progress_mm) * alpha);
            }
        }

        return bestProgressMm;
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

    float CMovelistexecutor::deciDegreesToRad(float deciDegrees)
    {
        return deciDegrees * (PI_FLOAT / 1800.0f);
    }

    float CMovelistexecutor::millimetersToMeters(float millimeters)
    {
        return millimeters / 1000.0f;
    }

    uint32_t CMovelistexecutor::fnv1aMix(uint32_t checksum, uint32_t value, uint8_t byteCount)
    {
        for (uint8_t index = 0U; index < byteCount; index++)
        {
            checksum ^= static_cast<uint8_t>((value >> (index * 8U)) & 0xFFu);
            checksum *= FNV_PRIME;
        }
        return checksum;
    }
}
