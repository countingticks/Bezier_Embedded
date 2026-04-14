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

#include <periodics/encoderdistancetest.hpp>

#include <brain/globalsv.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>

namespace periodics
{
    CEncoderDistanceTest::CEncoderDistanceTest(
            std::chrono::milliseconds f_period,
            drivers::CSerialTxBroker& f_serialBroker,
            CEncoder& f_encoder,
            drivers::ISteeringCommand& f_steeringControl,
            drivers::ISpeedingCommand& f_speedingControl
        )
        : utils::CTask(f_period)
        , m_serialBroker(f_serialBroker)
        , m_encoder(f_encoder)
        , m_steeringControl(f_steeringControl)
        , m_speedingControl(f_speedingControl)
        , m_isRunning(false)
        , m_speedCommandMmPerSec(0)
        , m_speedCommandCmPerSec(0)
        , m_elapsed(0)
        , m_testDuration(std::chrono::milliseconds(5000))
        , m_lastDistanceMm(0.0f)
    {
    }

    CEncoderDistanceTest::~CEncoderDistanceTest()
    {
    };

    void CEncoderDistanceTest::serialCallbackENCTESTcommand(char const * a, char * b)
    {
        int l_speedCmPerSec = 0;
        int l_durationSec = 0;
        const uint32_t l_res = sscanf(a, "%d;%d", &l_speedCmPerSec, &l_durationSec);

        if ((2 != l_res) || (l_durationSec <= 0))
        {
            sprintf(b, "syntax error");
            return;
        }

        if (uint8_globalsV_value_of_kl != 30)
        {
            sprintf(b, "kl 30 is required!!");
            return;
        }

        if (m_isRunning)
        {
            sprintf(b, "busy");
            return;
        }

        m_speedCommandMmPerSec = m_speedingControl.inRange(l_speedCmPerSec * 10);
        m_speedCommandCmPerSec = m_speedCommandMmPerSec / 10;
        m_testDuration = std::chrono::milliseconds(l_durationSec * 1000);
        m_elapsed = std::chrono::milliseconds(0);
        m_lastDistanceMm = 0.0f;

        m_encoder.resetTravelDistance();
        m_steeringControl.setAngle(0);
        m_speedingControl.setSpeed(m_speedCommandMmPerSec);

        m_isRunning = true;

        sprintf(b, "start;%d;%d", m_speedCommandCmPerSec, static_cast<int>(m_testDuration.count() / 1000));
    }

    void CEncoderDistanceTest::_run()
    {
        if (!m_isRunning)
        {
            return;
        }

        m_lastDistanceMm = m_encoder.getTravelDistanceMm();
        m_elapsed += m_period;

        if (m_elapsed >= m_testDuration)
        {
            finishTest();
        }
    }

    void CEncoderDistanceTest::finishTest()
    {
        char l_buffer[96];
        // Refresh once more so the reported distance includes the latest encoder
        // sample up to the stop decision.
        m_lastDistanceMm = m_encoder.getTravelDistanceMm();
        const int l_distanceMm = static_cast<int>(roundf(m_lastDistanceMm));
        const int l_averageSpeedMmPerSec = static_cast<int>(
            roundf((m_lastDistanceMm * 1000.0f) / static_cast<float>(m_testDuration.count()))
        );

        m_speedingControl.setBrake();
        m_steeringControl.setAngle(0);
        m_isRunning = false;

        snprintf(l_buffer, sizeof(l_buffer), "@encTest:%d;%d;;\r\n", l_distanceMm, l_averageSpeedMmPerSec);
        m_serialBroker.sendReliable(l_buffer, strlen(l_buffer));
    }
}; // namespace periodics
