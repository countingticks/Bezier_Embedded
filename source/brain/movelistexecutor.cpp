/**
 * @brief Move List Executor implementation.
 *
 *  Lifecycle:
 *    1. Pi sends N  "#moves:speed;steer;duration;;"  messages  → segments buffered
 *    2. Pi sends    "#moveGo:1;;"                               → execution starts
 *    3. Nucleo steps through each segment, applying speed/steer for its duration
 *    4. When all segments are done → motors zeroed, "@moveDone:1;;\r\n" sent back
 *    5. Pi can abort at any time with  "#moveStop:1;;"
 */

#include <brain/movelistexecutor.hpp>

namespace brain
{

    // ======================== CONSTRUCTOR ========================
    CMovelistexecutor::CMovelistexecutor(
            std::chrono::milliseconds   f_period,
            UnbufferedSerial&           f_serialPort,
            drivers::ISteeringCommand&  f_steeringControl,
            drivers::ISpeedingCommand&  f_speedingControl
        )
        : utils::CTask(f_period)
        , m_serialPort(f_serialPort)
        , m_steeringControl(f_steeringControl)
        , m_speedingControl(f_speedingControl)
        , m_moveCount(0)
        , m_executing(false)
        , m_currentMove(0)
        , m_ticksInMove(0)
        , m_period((uint16_t)(f_period.count()))
    {
    }

    CMovelistexecutor::~CMovelistexecutor()
    {
    };

    // ======================== PERIODIC RUN ========================
    void CMovelistexecutor::_run()
    {
        if (!m_executing) return;

        m_ticksInMove += m_period;

        // Check if current segment is finished
        if (m_ticksInMove >= m_moves[m_currentMove].duration_ms)
        {
            m_currentMove++;

            // All moves done?
            if (m_currentMove >= m_moveCount)
            {
                m_executing = false;
                m_speedingControl.setSpeed(0);
                m_steeringControl.setAngle(0);
                m_serialPort.write("@moveDone:1;;\r\n", 15);
                return;
            }

            // Apply next segment
            m_ticksInMove = 0;
            m_speedingControl.setSpeed(m_moves[m_currentMove].speed);
            m_steeringControl.setAngle(m_moves[m_currentMove].steer);
        }
    }

    // ======================== SERIAL CALLBACKS ========================

    /**
     * Append one move segment.
     * Message format: "speed;steer;duration"   (ints, semicolon-separated)
     * Example:        "150;-80;200"   → 150 mm/s, -8.0°, 200 ms
     */
    void CMovelistexecutor::serialCallbackMovesCommand(char const * message, char * response)
    {
        if (uint8_globalsV_value_of_kl != 30)
        {
            sprintf(response, "kl 30 is required!!");
            return;
        }

        int speed, steer, duration;
        uint8_t parsed = sscanf(message, "%d;%d;%d", &speed, &steer, &duration);

        if (parsed == 3 && m_moveCount < MAX_MOVES)
        {
            m_moves[m_moveCount].speed       = (int16_t)speed;
            m_moves[m_moveCount].steer        = (int16_t)steer;
            m_moves[m_moveCount].duration_ms  = (uint16_t)duration;
            m_moveCount++;
            sprintf(response, "%d", m_moveCount);
        }
        else if (m_moveCount >= MAX_MOVES)
        {
            sprintf(response, "full");
        }
        else
        {
            sprintf(response, "syntax error");
        }
    }

    /**
     * Start executing the buffered moves.
     * Message format: "1"
     */
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

        // Start from the first segment
        m_currentMove = 0;
        m_ticksInMove = 0;
        m_executing   = true;

        // Apply the first segment immediately
        m_speedingControl.setSpeed(m_moves[0].speed);
        m_steeringControl.setAngle(m_moves[0].steer);

        sprintf(response, "ok;%d", m_moveCount);
    }

    /**
     * Stop execution immediately and zero the motors.
     * Message format: "1"
     */
    void CMovelistexecutor::serialCallbackMoveStopCommand(char const * message, char * response)
    {
        m_executing = false;
        m_moveCount = 0;
        m_currentMove = 0;
        m_ticksInMove = 0;
        m_speedingControl.setSpeed(0);
        m_steeringControl.setAngle(0);
        sprintf(response, "stopped");
    }

}; // namespace brain
