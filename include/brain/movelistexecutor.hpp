/**
 * @brief Move List Executor - receives a batch of move segments from the Pi
 *        and executes them sequentially on the car's motors.
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
    /** @brief A single move segment: speed + steer held for a duration. */
    struct MoveSegment
    {
        int16_t  speed;       // mm/s
        int16_t  steer;       // degrees * 10
        uint16_t duration_ms; // how long to hold (ms)
    };

    /** Max number of move segments that can be queued. */
    static const uint8_t MAX_MOVES = 100;

    /**
     * @brief CMovelistexecutor stores an array of MoveSegments received over
     *        serial and executes them one-by-one, advancing when each segment's
     *        duration expires.
     */
    class CMovelistexecutor : public utils::CTask
    {
        public:
            /* Constructor */
            CMovelistexecutor(
                std::chrono::milliseconds   f_period,
                UnbufferedSerial&           f_serialPort,
                drivers::ISteeringCommand&  f_steeringControl,
                drivers::ISpeedingCommand&  f_speedingControl
            );

            /* Destructor */
            ~CMovelistexecutor();

            /** Serial callback: append a move segment.  Format: "speed;steer;duration;;" */
            void serialCallbackMovesCommand(char const * message, char * response);

            /** Serial callback: start executing the queued moves.  Format: "1;;" */
            void serialCallbackMoveGoCommand(char const * message, char * response);

            /** Serial callback: abort execution immediately.  Format: "1;;" */
            void serialCallbackMoveStopCommand(char const * message, char * response);

        private:
            /* Periodic run method */
            virtual void _run();

            /* References */
            UnbufferedSerial&           m_serialPort;
            drivers::ISteeringCommand&  m_steeringControl;
            drivers::ISpeedingCommand&  m_speedingControl;

            /* Move buffer */
            MoveSegment m_moves[MAX_MOVES];
            uint8_t     m_moveCount;

            /* Execution state */
            bool     m_executing;
            uint8_t  m_currentMove;
            uint16_t m_ticksInMove;   // ms elapsed in current segment
            uint16_t m_period;        // task period in ms

    }; // class CMovelistexecutor
}; // namespace brain

#endif // MOVELISTEXECUTOR_HPP
