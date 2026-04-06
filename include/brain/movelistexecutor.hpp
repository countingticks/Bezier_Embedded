/**
 * @brief Move List Executor - receives adaptive keyframes from the Pi and
 *        executes them with path-progress interpolation plus optional
 *        arrival hold at the final point.
 */

#ifndef MOVELISTEXECUTOR_HPP
#define MOVELISTEXECUTOR_HPP

#include <mbed.h>
#include <chrono>
#include <drivers/speedingmotor.hpp>
#include <drivers/steeringmotor.hpp>
#include <periodics/encoder.hpp>
#include <periodics/imu.hpp>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>

namespace brain
{
    struct PIDController
    {
        float kp;
        float ki;
        float kd;
        float integral;
        float previousError;
        float integralLimit;
        float outputLimit;
        bool initialized;

        PIDController(
            float f_kp = 0.0f,
            float f_ki = 0.0f,
            float f_kd = 0.0f,
            float f_integralLimit = 0.0f,
            float f_outputLimit = 0.0f
        )
            : kp(f_kp)
            , ki(f_ki)
            , kd(f_kd)
            , integral(0.0f)
            , previousError(0.0f)
            , integralLimit(f_integralLimit)
            , outputLimit(f_outputLimit)
            , initialized(false)
        {
        }

        void reset()
        {
            integral = 0.0f;
            previousError = 0.0f;
            initialized = false;
        }

        float update(float error, float dt);
    };

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
    static const uint32_t MOVE_PROGRESS_INTERVAL_MS = 50;

    class CMovelistexecutor : public utils::CTask
    {
        public:
            CMovelistexecutor(
                std::chrono::milliseconds   f_period,
                UnbufferedSerial&           f_serialPort,
                drivers::ISteeringCommand&  f_steeringControl,
                drivers::ISpeedingCommand&  f_speedingControl,
                periodics::CEncoder&        f_encoder,
                periodics::CImu&            f_imu
            );

            ~CMovelistexecutor();

            /** Serial callback: append one absolute-time keyframe. */
            void serialCallbackMovesCommand(char const * message, char * response);

            /** Serial callback: arm execution of the queued keyframes. */
            void serialCallbackMoveGoCommand(char const * message, char * response);

            /** Serial callback: abort execution immediately and clear the queue. */
            void serialCallbackMoveStopCommand(char const * message, char * response);

        private:
            virtual void _run();
            void applyInterpolatedCommand(uint32_t elapsed_ms);
            void interpolateCommandByTime(
                uint32_t elapsed_ms,
                float& speedFeedforward,
                float& steerFeedforward,
                float& referenceXmm,
                float& referenceYmm,
                float& referenceHeadingRad
            );
            void interpolateCommandByProgress(
                float progress_mm,
                float& speedFeedforward,
                float& steerFeedforward,
                float& referenceXmm,
                float& referenceYmm,
                float& referenceHeadingRad
            );
            float estimateProgressAlongTrajectory() const;
            bool hasReachedGoal() const;
            void updatePoseEstimate(float dt_s);
            void resetExecutionState(bool clearBuffer);
            void resetPoseEstimate();
            void sendProgressUpdate();
            static float clampFloat(float value, float minValue, float maxValue);
            static float wrapAngle(float angle);
            static float interpolateAngle(float startAngle, float endAngle, float alpha);
            static uint32_t fnv1aMix(uint32_t checksum, uint32_t value, uint8_t byteCount);

            UnbufferedSerial&           m_serialPort;
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
            uint32_t m_elapsedMs;
            uint32_t m_period;
            uint32_t m_lastProgressReportMs;
            uint32_t m_uploadChecksum;

            bool  m_poseReady;
            bool  m_headingInitialized;
            float m_initialYawDeg;
            float m_poseXmm;
            float m_poseYmm;
            float m_poseHeadingRad;
            float m_referenceXmm;
            float m_referenceYmm;
            float m_referenceHeadingRad;
            float m_referenceProgressMm;
            float m_estimatedProgressMm;
            float m_relativeXErrorMm;
            float m_relativeYErrorMm;
            float m_headingErrorRad;
            PIDController m_speedPid;
            PIDController m_lateralPid;
            float m_headingGain;
            int16_t m_lastCommandedSpeed;
            int16_t m_lastCommandedSteer;
    };
};

#endif // MOVELISTEXECUTOR_HPP

