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

#include <drivers/serialtxbroker.hpp>
#include <algorithm>
#include <cstring>

namespace drivers
{
    namespace
    {
        static const CSerialTxBroker::TelemetryTopic g_topicPriority[] = {
            CSerialTxBroker::TelemetryTopic::MoveProgress,
            CSerialTxBroker::TelemetryTopic::State,
            CSerialTxBroker::TelemetryTopic::Imu,
            CSerialTxBroker::TelemetryTopic::Encoder
        };
    }

    CSerialTxBroker::CSerialTxBroker(UnbufferedSerial& f_serialPort)
        : m_serialPort(f_serialPort)
        , m_reliableQueue()
        , m_reliableHead(0U)
        , m_reliableTail(0U)
        , m_reliableSize(0U)
        , m_telemetrySlots()
        , m_activeTelemetryBuffer()
        , m_activeTelemetryLength(0U)
        , m_activeTelemetryOffset(0U)
        , m_txIrqEnabled(false)
    {
    }

    CSerialTxBroker::~CSerialTxBroker()
    {
        CriticalSectionLock l_lock;
        setTxIrqEnabledLocked(false);
    }

    bool CSerialTxBroker::sendReliable(const char* f_data, size_t f_length)
    {
        if ((f_data == nullptr) || (f_length == 0U))
        {
            return true;
        }

        bool l_success = false;
        {
            CriticalSectionLock l_lock;
            if (f_length <= reliableFreeSpaceLocked())
            {
                l_success = true;
                for (size_t l_index = 0U; l_index < f_length; ++l_index)
                {
                    if (!pushReliableLocked(f_data[l_index]))
                    {
                        l_success = false;
                        break;
                    }
                }
            }
        }

        if (l_success)
        {
            kickTx();
        }

        return l_success;
    }

    bool CSerialTxBroker::publishLatest(TelemetryTopic f_topic, const char* f_data, size_t f_length)
    {
        if ((f_data == nullptr) || (f_length == 0U))
        {
            return true;
        }

        const size_t l_topicIndex = static_cast<size_t>(f_topic);
        if (l_topicIndex >= m_telemetrySlots.size())
        {
            return false;
        }

        TelemetrySlot& l_slot = m_telemetrySlots[l_topicIndex];
        const size_t l_copyLength = std::min(f_length, l_slot.buffer.size() - 1U);

        {
            CriticalSectionLock l_lock;
            if (l_slot.dirty && l_slot.length > 0U)
            {
                ++l_slot.overwriteCount;
            }
            std::memcpy(l_slot.buffer.data(), f_data, l_copyLength);
            l_slot.buffer[l_copyLength] = '\0';
            l_slot.length = static_cast<uint16_t>(l_copyLength);
            l_slot.dirty = true;
        }

        kickTx();
        return true;
    }

    uint32_t CSerialTxBroker::getOverwriteCount(TelemetryTopic f_topic) const
    {
        const size_t l_topicIndex = static_cast<size_t>(f_topic);
        if (l_topicIndex >= m_telemetrySlots.size())
        {
            return 0U;
        }

        CriticalSectionLock l_lock;
        return m_telemetrySlots[l_topicIndex].overwriteCount;
    }

    void CSerialTxBroker::serialTxCallback()
    {
        CriticalSectionLock l_lock;
        while (sendOneByteLocked())
        {
        }

        if (!hasPendingDataLocked())
        {
            setTxIrqEnabledLocked(false);
        }
    }

    void CSerialTxBroker::kickTx()
    {
        CriticalSectionLock l_lock;

        if (!hasPendingDataLocked())
        {
            setTxIrqEnabledLocked(false);
            return;
        }

        setTxIrqEnabledLocked(true);

        if (m_serialPort.writeable())
        {
            (void)sendOneByteLocked();
        }

        if (!hasPendingDataLocked())
        {
            setTxIrqEnabledLocked(false);
        }
    }

    void CSerialTxBroker::setTxIrqEnabledLocked(bool f_enabled)
    {
        if (f_enabled == m_txIrqEnabled)
        {
            return;
        }

        if (f_enabled)
        {
            m_serialPort.attach(mbed::callback(this, &CSerialTxBroker::serialTxCallback), SerialBase::TxIrq);
        }
        else
        {
            m_serialPort.attach(mbed::Callback<void()>(), SerialBase::TxIrq);
        }

        m_txIrqEnabled = f_enabled;
    }

    bool CSerialTxBroker::hasPendingDataLocked() const
    {
        if (m_reliableSize > 0U)
        {
            return true;
        }

        if (m_activeTelemetryOffset < m_activeTelemetryLength)
        {
            return true;
        }

        return hasDirtyTelemetryLocked();
    }

    bool CSerialTxBroker::hasDirtyTelemetryLocked() const
    {
        for (const TelemetrySlot& l_slot : m_telemetrySlots)
        {
            if (l_slot.dirty && l_slot.length > 0U)
            {
                return true;
            }
        }
        return false;
    }

    bool CSerialTxBroker::tryActivateNextTelemetryLocked()
    {
        if ((m_activeTelemetryOffset < m_activeTelemetryLength) || !hasDirtyTelemetryLocked())
        {
            return false;
        }

        for (TelemetryTopic l_topic : g_topicPriority)
        {
            TelemetrySlot& l_slot = m_telemetrySlots[static_cast<size_t>(l_topic)];
            if (!l_slot.dirty || (l_slot.length == 0U))
            {
                continue;
            }

            std::memcpy(m_activeTelemetryBuffer.data(), l_slot.buffer.data(), l_slot.length);
            m_activeTelemetryLength = l_slot.length;
            m_activeTelemetryOffset = 0U;
            l_slot.dirty = false;
            return true;
        }

        return false;
    }

    bool CSerialTxBroker::tryWriteByteLocked(char f_byte)
    {
        if (!m_serialPort.writeable())
        {
            return false;
        }

        const auto l_written = m_serialPort.write(&f_byte, 1);
        return l_written == 1;
    }

    bool CSerialTxBroker::sendOneByteLocked()
    {
        if (m_activeTelemetryOffset < m_activeTelemetryLength)
        {
            const char l_byte = m_activeTelemetryBuffer[m_activeTelemetryOffset];
            if (!tryWriteByteLocked(l_byte))
            {
                return false;
            }

            ++m_activeTelemetryOffset;
            if (m_activeTelemetryOffset >= m_activeTelemetryLength)
            {
                m_activeTelemetryLength = 0U;
                m_activeTelemetryOffset = 0U;
            }
            return true;
        }

        if (m_reliableSize > 0U)
        {
            const char l_byte = m_reliableQueue[m_reliableTail];
            if (!tryWriteByteLocked(l_byte))
            {
                return false;
            }

            m_reliableTail = (m_reliableTail + 1U) % m_reliableQueue.size();
            --m_reliableSize;
            return true;
        }

        if (!tryActivateNextTelemetryLocked())
        {
            return false;
        }

        const char l_byte = m_activeTelemetryBuffer[m_activeTelemetryOffset];
        if (!tryWriteByteLocked(l_byte))
        {
            return false;
        }

        ++m_activeTelemetryOffset;
        if (m_activeTelemetryOffset >= m_activeTelemetryLength)
        {
            m_activeTelemetryLength = 0U;
            m_activeTelemetryOffset = 0U;
        }
        return true;
    }

    bool CSerialTxBroker::pushReliableLocked(char f_byte)
    {
        if (m_reliableSize >= m_reliableQueue.size())
        {
            return false;
        }

        m_reliableQueue[m_reliableHead] = f_byte;
        m_reliableHead = (m_reliableHead + 1U) % m_reliableQueue.size();
        ++m_reliableSize;
        return true;
    }

    bool CSerialTxBroker::popReliableLocked(char& f_byte)
    {
        if (m_reliableSize == 0U)
        {
            return false;
        }

        f_byte = m_reliableQueue[m_reliableTail];
        m_reliableTail = (m_reliableTail + 1U) % m_reliableQueue.size();
        --m_reliableSize;
        return true;
    }

    size_t CSerialTxBroker::reliableFreeSpaceLocked() const
    {
        return m_reliableQueue.size() - m_reliableSize;
    }
}
