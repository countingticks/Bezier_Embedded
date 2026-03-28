/**
 * @brief Move List Executor - receives adaptive keyframes from the Pi and
 *        executes them with time-based interpolation.
 */

#ifndef MOVELISTEXECUTOR_HPP
#define MOVELISTEXECUTOR_HPP

#include <mbed.h>
#include <chrono>
#include <drivers/speedingmotor.hpp>
#include <drivers/steeringmotor.hpp>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>

namespace brain
{
    /** @brief A single keyframe: target speed/steer reached at absolute time_ms. */
    struct MoveSegment
    {
        int16_t  speed;   // mm/s
        int16_t  steer;   // degrees * 10
        uint32_t time_ms; // absolute time since moveGo start
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
                drivers::ISpeedingCommand&  f_speedingControl
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
            void resetExecutionState(bool clearBuffer);
            void sendProgressUpdate();
            static uint32_t fnv1aMix(uint32_t checksum, uint32_t value, uint8_t byteCount);

            UnbufferedSerial&           m_serialPort;
            drivers::ISteeringCommand&  m_steeringControl;
            drivers::ISpeedingCommand&  m_speedingControl;

            MoveSegment m_moves[MAX_MOVES];
            uint16_t    m_moveCount;

            bool     m_executing;
            uint16_t m_currentMove;
            uint32_t m_elapsedMs;
            uint32_t m_period;
            uint32_t m_lastProgressReportMs;
            uint32_t m_uploadChecksum;
    };
};

#endif // MOVELISTEXECUTOR_HPP

