/**
 * @brief Move List Executor - receives adaptive keyframes from the Pi and
 *        executes them with path-progress interpolation plus optional
 *        arrival hold at the final point.
 */

#ifndef MOVELISTEXECUTOR_HPP
#define MOVELISTEXECUTOR_HPP

#include <mbed.h>
#include <chrono>
#include <brain/mpccontroller.hpp>
#include <drivers/serialtxbroker.hpp>
#include <drivers/speedingmotor.hpp>
#include <drivers/steeringmotor.hpp>
#include <periodics/encoder.hpp>
#include <periodics/imu.hpp>
#include <utils/task.hpp>

namespace brain
{
    /** @brief A single keyframe: target speed/steer reached at absolute time_ms. */
    struct MoveSegment
    {
        int16_t  speed;            // mm/s
        int16_t  steer;            // degrees * 10
        uint32_t time_ms;          // absolute time since moveGo start
        int32_t  ref_x_mm;         // reference global x in millimeters
        int32_t  ref_y_mm;         // reference global y in millimeters
        int16_t  ref_heading_mrad; // reference heading in milliradians
        uint32_t progress_mm;      // cumulative path progress in millimeters
    };

    /** Max number of adaptive keyframes that can be queued. */
    static const uint16_t MAX_MOVES = 512;
    static const uint16_t MAX_DIRECTION_SEGMENTS = MAX_MOVES;
    static const uint32_t MOVE_PROGRESS_INTERVAL_DEFAULT_MS = 50;
    static const uint32_t MOVE_PROGRESS_INTERVAL_MIN_MS = 20;
    class CMovelistexecutor : public utils::CTask
    {
        public:
            CMovelistexecutor(
                std::chrono::milliseconds   f_period,
                drivers::CSerialTxBroker&   f_serialBroker,
                drivers::ISteeringCommand&  f_steeringControl,
                drivers::ISpeedingCommand&  f_speedingControl,
                periodics::CEncoder&        f_encoder,
                periodics::CImu&            f_imu
            );

            ~CMovelistexecutor() = default;

            /** Serial callback: append one absolute-time keyframe. */
            void serialCallbackMovesCommand(char const * message, char * response);

            /** Serial callback: arm execution of the queued keyframes. */
            void serialCallbackMoveGoCommand(char const * message, char * response);

            /** Serial callback: abort execution immediately and clear the queue. */
            void serialCallbackMoveStopCommand(char const * message, char * response);

            struct ExecutorPoseSnapshot
            {
                float x_mm;
                float y_mm;
                float heading_rad;
                float matched_progress_mm;
                float raw_projected_progress_mm;
                float odometry_progress_mm;
                uint32_t seq;
                bool valid;
                bool executing;

                ExecutorPoseSnapshot()
                    : x_mm(0.0f)
                    , y_mm(0.0f)
                    , heading_rad(0.0f)
                    , matched_progress_mm(0.0f)
                    , raw_projected_progress_mm(0.0f)
                    , odometry_progress_mm(0.0f)
                    , seq(0U)
                    , valid(false)
                    , executing(false)
                {
                }
            };

            ExecutorPoseSnapshot getPoseSnapshot() const
            {
                return m_poseSnapshot;
            }

        private:
            void _run() override;
            void applyInterpolatedCommand(uint32_t elapsed_ms);
            void interpolateCommandByTime(
                uint32_t elapsed_ms,
                float& speedFeedforward,
                float& steerFeedforward,
                float& referenceXmm,
                float& referenceYmm,
                float& referenceHeadingRad,
                float& referenceProgressMm
            );
            void interpolateCommandByProgress(
                float progress_mm,
                float& speedFeedforward,
                float& steerFeedforward,
                float& referenceXmm,
                float& referenceYmm,
                float& referenceHeadingRad,
                float& referenceProgressMm
            );
            float estimateProgressAlongTrajectory(
                uint16_t directionSegmentIndex,
                float& bestDistanceSquared,
                bool& projectionValid,
                uint16_t& projectedMoveIndex
            );
            bool hasReachedGoal() const;
            void updatePoseEstimate();
            void refreshPoseSnapshot();
            void resetExecutionState(bool clearBuffer);
            void resetPoseEstimate();
            void resetControllerState();
            void sendProgressUpdate();
            void rebuildDirectionSegments();
            bool ensureActiveDirectionSegment(uint32_t elapsedMs, uint16_t& directionSegmentIndex);
            void activateDirectionSegment(uint16_t directionSegmentIndex);
            void fillMpcHorizon(
                uint32_t elapsedMs,
                float progressSeedMm,
                uint16_t directionSegmentIndex,
                float currentSpeedFeedforwardMmS,
                float currentSteerFeedforwardDeciDeg,
                CMpcController::HorizonSample (&horizon)[CMpcController::c_horizonLength]
            );
            CMpcController::Limits makeMpcLimits(int8_t direction, float referencePathSpeedMps);
            bool getDynamicSegmentEndpoint(
                uint16_t directionSegmentIndex,
                uint16_t& endpointIndex,
                uint32_t& endpointTimeMs,
                float& endpointProgressMm
            ) const;
            uint16_t getMoveIndexForProgress(uint16_t directionSegmentIndex, float progressMm) const;
            float getSegmentStartProgressMm(uint16_t directionSegmentIndex) const;
            float getSegmentLimitProgressMm(uint16_t directionSegmentIndex) const;
            uint16_t getProjectionSearchEndMoveIndex(uint16_t directionSegmentIndex, uint16_t acceptedMoveIndex) const;
            float computeMatchedProgressMm(
                uint16_t directionSegmentIndex,
                float nominalProgressMm,
                float rawProjectedProgressMm,
                float projectedDistanceMm,
                bool projectionValid
            );
            float computeHorizonSeedProgressMm(uint16_t directionSegmentIndex, float nominalProgressMm) const;
            static int8_t classifyDirection(int16_t startSpeed, int16_t endSpeed);
            float estimateProgressInRange(uint16_t startIndex, uint16_t endIndex, float& bestDistanceSquared, uint16_t& bestSegment) const;
            uint16_t getCheckpointWindowEndMoveIndex(uint16_t directionSegmentIndex, uint16_t acceptedMoveIndex) const;
            static float clampFloat(float value, float minValue, float maxValue);
            static float wrapAngle(float angle);
            static float interpolateAngle(float startAngle, float endAngle, float alpha);
            static float deciDegreesToRad(float deciDegrees);
            static float millimetersToMeters(float millimeters);
            static uint32_t fnv1aMix(uint32_t checksum, uint32_t value, uint8_t byteCount);

            struct DirectionSegment
            {
                uint16_t start_index;
                uint16_t end_index;
                uint32_t time_start_ms;
                uint32_t time_end_ms;
                uint32_t progress_start_mm;
                uint32_t progress_end_mm;
                int8_t direction;
            };

            drivers::CSerialTxBroker&   m_serialBroker;
            drivers::ISteeringCommand&  m_steeringControl;
            drivers::ISpeedingCommand&  m_speedingControl;
            periodics::CEncoder&        m_encoder;
            periodics::CImu&            m_imu;

            MoveSegment m_moves[MAX_MOVES];
            uint16_t    m_moveCount;
            bool        m_hasReferenceTrajectory;
            bool        m_feedbackEnabled;
            bool        m_holdFinalUntilArrival;
            uint32_t    m_arrivalToleranceMm;

            bool     m_executing;
            uint16_t m_currentMove;
            uint16_t m_acceptedMoveIndex;
            uint16_t m_projectedMoveIndex;
            uint32_t m_elapsedMs;
            Kernel::Clock::time_point m_executionStartTime;
            Kernel::Clock::time_point m_lastRunTime;
            uint16_t m_directionSegmentCount;
            uint16_t m_activeDirectionSegment;
            uint32_t m_nextMpcSolveMs;
            uint32_t m_lastProgressReportMs;
            uint32_t m_progressReportIntervalMs;
            uint32_t m_uploadChecksum;
            uint32_t m_lastExecutorLoopDtMs;
            uint32_t m_maxExecutorLoopDtMs;
            uint32_t m_lastMpcSolveUs;
            uint32_t m_maxMpcSolveUs;
            uint32_t m_missedMpcSolveSlots;
            uint32_t m_poseSnapshotSeq;
            uint32_t m_reverseTravelCount;
            uint32_t m_yawJumpCount;
            uint32_t m_lastHorizonSeedTimeMs;

            bool  m_poseReady;
            bool  m_headingInitialized;
            float m_lastYawDeg;
            float m_lastImuYawDeg;
            float m_accumulatedYawDeltaRad;
            float m_poseXmm;
            float m_poseYmm;
            float m_poseHeadingRad;
            float m_encoderTravelMm;
            float m_lastEncoderTravelDeltaMm;
            float m_lastTravelDeltaMm;
            float m_reportTravelDeltaMm;
            float m_reportedEncoderTravelMm;
            float m_odometryProgressMm;
            float m_referenceXmm;
            float m_referenceYmm;
            float m_referenceHeadingRad;
            float m_referenceProgressMm;
            float m_rawProjectedProgressMm;
            float m_matchedProgressMm;
            float m_lastValidProjectedProgressMm;
            float m_relativeXErrorMm;
            float m_relativeYErrorMm;
            float m_headingErrorRad;
            float m_mpcProgressErrorM;
            float m_mpcLateralErrorM;
            float m_mpcHeadingErrorRad;
            float m_cachedPathCorrectionMps;
            float m_cachedSteerCorrectionRad;
            float m_lastNominalProgressMm;
            float m_lastTrustedTravelSpeedMmS;
            float m_lastReferenceSpeedMmS;
            float m_lastReferenceSteerDeciDeg;
            float m_lastReferenceCurvatureInvM;
            float m_lastHorizonSeedProgressMm;
            float m_lastHorizonReferenceSpeedMmS;
            float m_lastHorizonReferenceSteerDeciDeg;
            float m_lastPreClampCommandSpeedMmS;
            float m_lastPreClampCommandSteerDeciDeg;
            float m_lastSpeedCorrectionMps;
            float m_lastSteerCorrectionRad;
            float m_lastPathSpeedCommandMps;
            float m_lastPathSpeedLimitMps;
            float m_lastSteerCommandRad;
            float m_lastMpcObjective;
            float m_lastMpcMaxConstraintViolation;
            CMpcController::SolverStatus m_lastMpcStatus;
            uint16_t m_lastMpcIterations;
            uint16_t m_lastReferenceSegmentIndex;
            bool m_lastUsedPreviousCorrection;
            bool m_lastUsedFeedforwardOnly;
            bool m_lastCheckpointRecoveryHold;
            bool m_lastSpeedSaturated;
            bool m_lastSteerSaturated;
            bool m_lastSpeedRateLimited;
            bool m_lastSteerRateLimited;
            bool m_projectionValid;
            uint16_t m_projectionStaleCount;
            uint16_t m_lastRecoverySeverityPermille;
            CMpcController m_mpcController;
            DirectionSegment m_directionSegments[MAX_DIRECTION_SEGMENTS];
            int16_t m_lastCommandedSpeed;
            int16_t m_lastCommandedSteer;
            ExecutorPoseSnapshot m_poseSnapshot;
    };
};

#endif // MOVELISTEXECUTOR_HPP

