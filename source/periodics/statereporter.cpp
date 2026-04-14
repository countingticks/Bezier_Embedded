/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#include <periodics/statereporter.hpp>
#include <brain/globalsv.hpp>
#include <cmath>
#include <cstring>
#include <cstdio>

namespace
{
    constexpr uint32_t c_flagYawValid = 1U << 0;
    constexpr uint32_t c_flagVehicleMoving = 1U << 1;
    constexpr float c_motionThresholdMmS = 5.0f;

    int roundToInt(float f_value)
    {
        return static_cast<int>(f_value >= 0.0f ? f_value + 0.5f : f_value - 0.5f);
    }
}

namespace periodics
{
    CStateReporter::CStateReporter(
        std::chrono::milliseconds f_period,
        drivers::CSerialTxBroker& f_serialBroker,
        CEncoder& f_encoder,
        CImu& f_imu,
        drivers::ISteeringCommand& f_steeringControl
    )
        : utils::CTask(f_period)
        , m_serialBroker(f_serialBroker)
        , m_encoder(f_encoder)
        , m_imu(f_imu)
        , m_steeringControl(f_steeringControl)
        , m_isActive(false)
        , m_publishAccumulatorMs(0U)
        , m_reportIntervalMs(c_defaultReportIntervalMs)
    {
    }

    void CStateReporter::serialCallbackSTATEcommand(char const* a, char* b)
    {
        unsigned int l_isActivate = 0U;
        unsigned int l_reportIntervalMs = c_defaultReportIntervalMs;
        const int l_res = sscanf(a, "%u;%u", &l_isActivate, &l_reportIntervalMs);

        if (l_res == 1 || l_res == 2)
        {
            if (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                if (l_res == 2)
                {
                    if (l_reportIntervalMs < c_minReportIntervalMs)
                    {
                        l_reportIntervalMs = c_minReportIntervalMs;
                    }
                    if (l_reportIntervalMs > c_maxReportIntervalMs)
                    {
                        l_reportIntervalMs = c_maxReportIntervalMs;
                    }
                    m_reportIntervalMs = l_reportIntervalMs;
                }

                m_isActive = (l_isActivate >= 1U);
                sprintf(b, "1");
            }
            else
            {
                sprintf(b, "kl 15/30 is required!!");
            }
        }
        else
        {
            sprintf(b, "syntax error");
        }
    }

    void CStateReporter::_run()
    {
        if (!m_isActive)
        {
            return;
        }

        m_publishAccumulatorMs += static_cast<uint32_t>(m_period.count());
        if (m_publishAccumulatorMs < m_reportIntervalMs)
        {
            return;
        }

        m_publishAccumulatorMs = 0U;
        publishState();
    }

    void CStateReporter::publishState()
    {
        char l_buffer[96];
        const int l_speedMmS = roundToInt(m_encoder.getLinearSpeed());
        const int l_yawMrad = m_imu.hasValidYaw() ? roundToInt(m_imu.getYawDegrees() * 17.45329252f) : 0;
        const int l_steerDeciDeg = m_steeringControl.getCommandedAngle();
        uint32_t l_flags = 0U;

        if (m_imu.hasValidYaw())
        {
            l_flags |= c_flagYawValid;
        }
        if (std::fabs(static_cast<float>(l_speedMmS)) > c_motionThresholdMmS)
        {
            l_flags |= c_flagVehicleMoving;
        }

        snprintf(
            l_buffer,
            sizeof(l_buffer),
            "@state:%d;%d;%d;%lu;;\r\n",
            l_speedMmS,
            l_yawMrad,
            l_steerDeciDeg,
            static_cast<unsigned long>(l_flags)
        );
        m_serialBroker.publishLatest(drivers::CSerialTxBroker::TelemetryTopic::State, l_buffer, strlen(l_buffer));
    }
}
