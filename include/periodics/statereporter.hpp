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

#ifndef STATE_REPORTER_HPP
#define STATE_REPORTER_HPP

#include <mbed.h>
#include <chrono>
#include <drivers/serialtxbroker.hpp>
#include <drivers/steeringmotor.hpp>
#include <periodics/encoder.hpp>
#include <periodics/imu.hpp>
#include <utils/task.hpp>

namespace periodics
{
    class CStateReporter : public utils::CTask
    {
        public:
            CStateReporter(
                std::chrono::milliseconds f_period,
                drivers::CSerialTxBroker& f_serialBroker,
                CEncoder& f_encoder,
                CImu& f_imu,
                drivers::ISteeringCommand& f_steeringControl
            );
            ~CStateReporter() = default;

            void serialCallbackSTATEcommand(char const* a, char* b);

        private:
            void _run() override;
            void publishState();

            drivers::CSerialTxBroker& m_serialBroker;
            CEncoder& m_encoder;
            CImu& m_imu;
            drivers::ISteeringCommand& m_steeringControl;
            bool m_isActive;
            uint32_t m_publishAccumulatorMs;
            uint32_t m_reportIntervalMs;

            static constexpr uint32_t c_defaultReportIntervalMs = 100U;
            static constexpr uint32_t c_minReportIntervalMs = 100U;
            static constexpr uint32_t c_maxReportIntervalMs = 150U;
    };
}

#endif // STATE_REPORTER_HPP
