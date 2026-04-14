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

#ifndef SERIAL_TX_BROKER_HPP
#define SERIAL_TX_BROKER_HPP

#include <mbed.h>
#include <array>
#include <cstddef>
#include <cstdint>

namespace drivers
{
    class CSerialTxBroker
    {
        public:
            static constexpr size_t c_telemetryMessageCapacity = 640U;

            enum class TelemetryTopic : uint8_t
            {
                MoveProgress = 0U,
                State,
                Imu,
                Encoder,
                Count
            };

            explicit CSerialTxBroker(UnbufferedSerial& f_serialPort);
            ~CSerialTxBroker();

            bool sendReliable(const char* f_data, size_t f_length);
            bool publishLatest(TelemetryTopic f_topic, const char* f_data, size_t f_length);
            uint32_t getOverwriteCount(TelemetryTopic f_topic) const;

        private:
            struct TelemetrySlot
            {
                std::array<char, c_telemetryMessageCapacity> buffer;
                uint16_t length;
                bool dirty;
                uint32_t overwriteCount;

                TelemetrySlot()
                    : buffer()
                    , length(0U)
                    , dirty(false)
                    , overwriteCount(0U)
                {
                }
            };

            void serialTxCallback();
            void kickTx();
            void setTxIrqEnabledLocked(bool f_enabled);
            bool hasPendingDataLocked() const;
            bool hasDirtyTelemetryLocked() const;
            bool tryActivateNextTelemetryLocked();
            bool tryWriteByteLocked(char f_byte);
            bool sendOneByteLocked();
            bool pushReliableLocked(char f_byte);
            bool popReliableLocked(char& f_byte);
            size_t reliableFreeSpaceLocked() const;

            UnbufferedSerial& m_serialPort;

            static constexpr size_t c_reliableQueueCapacity = 2048U;
            std::array<char, c_reliableQueueCapacity> m_reliableQueue;
            size_t m_reliableHead;
            size_t m_reliableTail;
            size_t m_reliableSize;

            std::array<TelemetrySlot, static_cast<size_t>(TelemetryTopic::Count)> m_telemetrySlots;
            std::array<char, c_telemetryMessageCapacity> m_activeTelemetryBuffer;
            uint16_t m_activeTelemetryLength;
            uint16_t m_activeTelemetryOffset;
            bool m_txIrqEnabled;
    };
}

#endif // SERIAL_TX_BROKER_HPP
