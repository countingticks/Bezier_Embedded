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

/* Include guard */
#ifndef ENCODER_HPP
#define ENCODER_HPP

/* The mbed library */
#include <mbed.h>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/task.hpp>
#include <chrono>
#include <cstddef>

namespace periodics
{
    struct CEncoderKalman2D
    {
        float angle;
        float speed;

        float p00;
        float p01;
        float p11;

        float q_angle;
        float q_speed;
        float r_angle;

        CEncoderKalman2D();

        void predict(float f_dt);
        void update(float f_measurement);
    };

    class CEncoder : public utils::CTask
    {
        public:
            /* Constructor */
            CEncoder(
                std::chrono::milliseconds f_period,
                UnbufferedSerial& f_serial,
                PinName f_sdaPin,
                PinName f_sclPin
            );
            /* Destructor */
            ~CEncoder();

            /* Serial callback implementation */
            void serialCallbackENCODERcommand(char const * a, char * b);
            /* Update the expected linear speed reference in mm/s */
            void setSpeedReference(float f_speed);

            float readAngleDegrees();
            float readAngularSpeed();
            float readAngularAcceleration();
            float getTotalDisplacementDegrees();
            void resetTravelDistance();
            float getTravelDistanceMm();
            float getLinearSpeed();
            float getLinearAcceleration();

        private:
            /* Run method */
            virtual void _run();

            float readAngularSpeedKalman();
            float getRawAngleDegrees();
            float applyHampel(float f_newSample);
            float applyHysteresis(float f_angle);
            float applySpeedHysteresis(float f_speed);
            float applyReferenceFilter(float f_speedMmPerSec) const;
            float convertAngularToLinear(float f_angularQuantity) const;
            void publishSpeed();

            struct CBiquad
            {
                float b0;
                float b1;
                float b2;
                float a1;
                float a2;
                float z1;
                float z2;

                CBiquad();
                float process(float f_input);
                void reset();
            };

            static constexpr size_t c_hampelWindow = 7;
            static constexpr size_t c_speedReferenceHistorySize = 5;

            /* AS5600 encoder on I2C */
            I2C m_i2c;
            /* @brief Serial communication obj.  */
            UnbufferedSerial& m_serial;
            /* @brief Active flag  */
            bool m_isActive;
            /* @brief Expected speed command in mm/s */
            float m_speedReference;
            /* @brief Recent speed commands used by the reference gate */
            float m_speedReferenceHistory[c_speedReferenceHistorySize];
            /* @brief Last published angular speed in deg/s */
            float m_lastAngularSpeed;
            /* @brief Last published angular acceleration in deg/s² */
            float m_lastAngularAcceleration;
            /* @brief Last published linear speed in mm/s */
            float m_lastLinearSpeed;
            /* @brief Last published linear acceleration in mm/s² */
            float m_lastLinearAcceleration;
            /* @brief Last raw encoder angle for unwrap */
            float m_previousRawAngle;
            /* @brief Last valid raw angle measurement in degrees */
            float m_lastRawAngleDegrees;
            /* @brief Revolution counter for unwrap */
            int m_unwrapRevolutions;
            /* @brief Publish accumulator in seconds */
            float m_publishAccumulator;
            /* @brief Last timer value in microseconds */
            std::chrono::microseconds::rep m_lastTimerUs;
            /* @brief Total displacement in degrees */
            float m_totalDisplacement;
            /* @brief First displacement sample marker */
            bool m_hasDisplacementReference;
            /* @brief Last displacement raw angle */
            float m_lastDisplacementRawAngle;
            /* @brief Speed timer started marker */
            bool m_timerStarted;

            float m_hampelBuffer[c_hampelWindow];
            size_t m_hampelIndex;
            size_t m_hampelCount;

            float m_samplingFrequency;
            float m_angleHysteresis;
            float m_speedHysteresis;
            float m_lastAngle;
            float m_reportInterval;

            Timer m_timer;
            CEncoderKalman2D m_kalman;
            CBiquad m_sinFilter;
            CBiquad m_cosFilter;

            static constexpr int c_as5600Address = 0x36 << 1;
            static constexpr char c_rawAngleHighRegister = 0x0C;
    }; // class CEncoder
}; // namespace periodics

#endif // ENCODER_HPP
