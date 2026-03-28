/**
 * @brief Move List Executor implementation.
 *
 * Lifecycle:
 *   1. Pi sends N  "#moves:speed;steer;time_ms;;" messages -> keyframes buffered
 *   2. Pi sends    "#moveGo:1;;"                        -> execution starts
 *   3. Nucleo interpolates speed/steer between keyframes on every task tick
 *   4. Progress is reported periodically until "@moveDone:<elapsed_ms>;;"
 *   5. Pi can abort at any time with "#moveStop:1;;"
 */

#include <brain/movelistexecutor.hpp>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
    static const uint32_t FNV_OFFSET_BASIS = 2166136261u;
    static const uint32_t FNV_PRIME = 16777619u;

    static int16_t roundToInt16(double value)
    {
        return static_cast<int16_t>(value >= 0.0 ? value + 0.5 : value - 0.5);
    }
}

namespace brain
{
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
        , m_elapsedMs(0)
        , m_period(static_cast<uint32_t>(f_period.count()))
        , m_lastProgressReportMs(0)
        , m_uploadChecksum(FNV_OFFSET_BASIS)
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

        if (m_elapsedMs < totalTimeMs)
        {
            m_elapsedMs += m_period;
            if (m_elapsedMs > totalTimeMs)
            {
                m_elapsedMs = totalTimeMs;
            }
        }

        while (m_currentMove + 1 < m_moveCount && m_elapsedMs >= m_moves[m_currentMove + 1].time_ms)
        {
            m_currentMove++;
        }

        applyInterpolatedCommand(m_elapsedMs);

        if ((m_elapsedMs - m_lastProgressReportMs) >= MOVE_PROGRESS_INTERVAL_MS || m_elapsedMs >= totalTimeMs)
        {
            sendProgressUpdate();
            m_lastProgressReportMs = m_elapsedMs;
        }

        if (m_elapsedMs >= totalTimeMs)
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

        int speed, steer;
        unsigned long timeMs;
        uint8_t parsed = sscanf(message, "%d;%d;%lu", &speed, &steer, &timeMs);

        if (parsed != 3)
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
        }
        else if (timeMs <= m_moves[m_moveCount - 1].time_ms)
        {
            sprintf(response, "time order");
            return;
        }

        m_moves[m_moveCount].speed = static_cast<int16_t>(speed);
        m_moves[m_moveCount].steer = static_cast<int16_t>(steer);
        m_moves[m_moveCount].time_ms = static_cast<uint32_t>(timeMs);
        m_uploadChecksum = fnv1aMix(m_uploadChecksum, static_cast<uint16_t>(m_moves[m_moveCount].speed), 2);
        m_uploadChecksum = fnv1aMix(m_uploadChecksum, static_cast<uint16_t>(m_moves[m_moveCount].steer), 2);
        m_uploadChecksum = fnv1aMix(m_uploadChecksum, m_moves[m_moveCount].time_ms, 4);
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

        if (m_moveCount == 1 || m_currentMove + 1 >= m_moveCount)
        {
            m_speedingControl.setSpeed(m_moves[m_moveCount - 1].speed);
            m_steeringControl.setAngle(m_moves[m_moveCount - 1].steer);
            return;
        }

        const MoveSegment& startFrame = m_moves[m_currentMove];
        const MoveSegment& endFrame = m_moves[m_currentMove + 1];

        if (endFrame.time_ms <= startFrame.time_ms)
        {
            m_speedingControl.setSpeed(endFrame.speed);
            m_steeringControl.setAngle(endFrame.steer);
            return;
        }

        double alpha = static_cast<double>(elapsed_ms - startFrame.time_ms) /
                       static_cast<double>(endFrame.time_ms - startFrame.time_ms);
        if (alpha < 0.0)
        {
            alpha = 0.0;
        }
        if (alpha > 1.0)
        {
            alpha = 1.0;
        }

        const double speed = static_cast<double>(startFrame.speed) +
                             (static_cast<double>(endFrame.speed - startFrame.speed) * alpha);
        const double steer = static_cast<double>(startFrame.steer) +
                             (static_cast<double>(endFrame.steer - startFrame.steer) * alpha);

        m_speedingControl.setSpeed(roundToInt16(speed));
        m_steeringControl.setAngle(roundToInt16(steer));
    }

    void CMovelistexecutor::resetExecutionState(bool clearBuffer)
    {
        m_executing = false;
        m_currentMove = 0;
        m_elapsedMs = 0;
        m_lastProgressReportMs = 0;
        if (clearBuffer)
        {
            m_moveCount = 0;
            m_uploadChecksum = FNV_OFFSET_BASIS;
        }
    }

    void CMovelistexecutor::sendProgressUpdate()
    {
        char buffer[64];
        snprintf(
            buffer,
            sizeof(buffer),
            "@moveProgress:%lu;%u;;\r\n",
            static_cast<unsigned long>(m_elapsedMs),
            static_cast<unsigned>(m_currentMove)
        );
        m_serialPort.write(buffer, strlen(buffer));
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

