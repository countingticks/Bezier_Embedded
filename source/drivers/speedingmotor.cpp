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

#include <drivers/speedingmotor.hpp>
#include <periodics/encoder.hpp>

#define calibrated 1
#define calib_sup_limit 500
#define calib_inf_limit -500

namespace drivers{
    /**
     * @brief It initializes the pwm parameters and it sets the speed reference to zero position, and the limits of the car speed.
     *
     * @param f_pwm_pin               pin connected to servo motor
     * @param f_inf_limit         inferior limit
     * @param f_sup_limit         superior limit
     *
     */
    CSpeedingMotor::CSpeedingMotor(
            PinName f_pwm_pin,
            int f_inf_limit,
            int f_sup_limit,
            periodics::CEncoder& f_encoder
        )
        : m_pwm_pin(f_pwm_pin)
        , m_inf_limit(f_inf_limit)
        , m_sup_limit(f_sup_limit)
        , m_encoder(f_encoder)
    {
        // Set the ms_period on the pwm_pin
        m_pwm_pin.period_ms(ms_period);
        // Set position to zero
        m_pwm_pin.pulsewidth_us(zero_default);
    };

    /** @brief  CSpeedingMotor class destructor
     */
    CSpeedingMotor::~CSpeedingMotor()
    {
    };

    /** @brief  It modifies the speed reference of the brushless motor, which controls the speed of the wheels.
     *
     *  @param f_speed      speed in mm/s, where the positive value means forward direction and negative value the backward direction.
     */
    void CSpeedingMotor::setSpeed(int f_speed)
    {
        pwm_value = zero_default;

        if (f_speed != 0) {
            if (calibrated == 1)
            {
                pwm_value = computePWMPolynomial(f_speed);
            }
            else {
                pwm_value = interpolate(-f_speed, speedValuesP, speedValuesN, pwmValuesP, pwmValuesN, 25);
            }
        }

        m_pwm_pin.pulsewidth_us(pwm_value);
    };

    /** @brief  It converts speed reference to duty cycle for pwm signal.
     *
     *  @param f_speed    speed
     *  \return        new `pwm_value`
    */
    int CSpeedingMotor::computePWMPolynomial(int speed)
    {
        int64_t y=zero_default;
        // POLYNOMIAL CODE START

        // Cubic spline evaluation with 14 segments
        static const int64_t knots[15] = {-550, -479, -421, -296, -196, -93, -49, 0, 52, 101, 205, 301, 406, 504, 550};
        static const int64_t coeffs[14][4] = { 
            {0LL, 18LL, -75406LL, 1707081728LL},
            {-3LL, 213LL, -73079LL, 1701838848LL},
            {-1LL, -83LL, -82360LL, 1697644544LL},
            {3LL, -701LL, -127219LL, 1685061632LL},
            {-1LL, -47LL, -180141LL, 1668284416LL},
            {-75LL, 2804LL, -217165LL, 1648361472LL},
            {473LL, -45327LL, -405418LL, 1637875712LL},
            {295LL, -13735LL, -1393872LL, 1563426816LL},
            {-51LL, 5923LL, -421857LL, 1495269376LL},
            {-1LL, 487LL, -209915LL, 1482686464LL},
            {-1LL, 254LL, -148562LL, 1464860672LL},
            {0LL, 117LL, -119395LL, 1452277760LL},
            {-2LL, 226LL, -102442LL, 1440743424LL},
            {2LL, -272LL, -106108LL, 1431306240LL}
        };
        
        // Find the correct segment
        int segment = -1;
        for (int i = 0; i < 14; i++) {
            if (speed >= knots[i] && speed <= knots[i + 1]) {
                segment = i;
                break;
            }
        }
        
        // Fall back to the boundary segment if the knot search did not find an exact match
        if (segment == -1) {
            if (speed < knots[0]) segment = 0;
            else segment = 14 - 1;
        }
        
        // Evaluate cubic polynomial for this segment: a*(x-xi)^3 + b*(x-xi)^2 + c*(x-xi) + d
        int64_t dx = speed - knots[segment];
        int64_t dx2 = dx * dx;
        int64_t dx3 = dx2 * dx;
        
        y = (coeffs[segment][0] * dx3 + coeffs[segment][1] * dx2 + 
             coeffs[segment][2] * dx + coeffs[segment][3]) / 1048576LL;
        
        /* Cubic spline interpolation with 14 segments
         * Each segment is defined by: a*(x-xi)^3 + b*(x-xi)^2 + c*(x-xi) + d
         * Coefficients are scaled by 1048576 for integer arithmetic
         */
        // POLYNOMIAL CODE END
        return (int)y;
    }

    /** @brief  It puts the brushless motor into brake state,
     */
    void CSpeedingMotor::setBrake()
    {
        m_pwm_pin.pulsewidth_us(zero_default);
    };

    /**
    * @brief Interpolates values based on speed input.
    *
    * This function interpolates `pwmValues` based on the provided `speed` input.
    * The interpolation is made using `speedValuesP` and `speedValuesN` as reference values.
    *
    * @param speed The input speed value for which the values need to be interpolated.
    * @param speedValuesP Positive reference values for speed.
    * @param speedValuesN Negative reference values for speed.
    * @param pwmValuesP PWM values corresponding to speedValueP
    * @param pwmValuesN PWM values corresponding to speedValueN
    * @param size The size of the arrays.
    * @return The new value for `pwm_value`
    */
    int16_t CSpeedingMotor::interpolate(int speed, const int speedValuesP[], const int speedValuesN[], const int pwmValuesP[], const int pwmValuesN[], int size)
    {
        const int SCALE = 1000; // Precision factor for fixed-point arithmetic

        if(speed == 0) return zero_default;
        if(speed >= speedValuesP[size-1]) return pwmValuesP[size-1];
        if(speed <= speedValuesN[size-1]) return pwmValuesN[size-1];

        // For negative speed values
        if(speed <= speedValuesP[0]){
            if(speed > 0) return pwmValuesP[0];
            if (speed >= speedValuesN[0])
            {
                return pwmValuesN[0];
            }
            else {
                for(uint8_t i = 1; i < size; i++)
                {
                    if (speed >= speedValuesN[i])
                    {
                        int deltaPWM = (pwmValuesN[i] - pwmValuesN[i-1]) * SCALE;
                        int deltaSpeed = speedValuesN[i] - speedValuesN[i-1];
                        int slope = deltaPWM / deltaSpeed; // Compute slope in fixed-point
                        int interpFixed = pwmValuesN[i-1] * SCALE + slope * (speed - speedValuesN[i-1]);
                        return (int16_t)(interpFixed / SCALE);
                    }
                }
            }
        }

        // For positive speed values
        for(uint8_t i = 1; i < size; i++)
        {
            if (speed <= speedValuesP[i])
            {
                int deltaPWM = (pwmValuesP[i] - pwmValuesP[i-1]) * SCALE;
                int deltaSpeed = speedValuesP[i] - speedValuesP[i-1];
                int slope = deltaPWM / deltaSpeed; // Compute slope in fixed-point
                int interpFixed = pwmValuesP[i-1] * SCALE + slope * (speed - speedValuesP[i-1]);
                return (int16_t)(interpFixed / SCALE);
            }
        }

        return zero_default;
    }

    /**
     * @brief It verifies whether a number is in a given range
     *
     * @param f_speed value
     * @return inf_limit, if the value is lower than the range's low
     * @return sup_limit, if the value is higher than the range's high
    */
    int CSpeedingMotor::inRange(int f_speed){

        if(calibrated == 1){
            if(f_speed < calib_inf_limit) return calib_inf_limit;
            if(f_speed > calib_sup_limit) return calib_sup_limit;
            return f_speed;
        } else{
            if(f_speed < m_inf_limit) return m_inf_limit;
            if(f_speed > m_sup_limit) return m_sup_limit;
            return f_speed;
        }

    };

    int CSpeedingMotor::get_upper_limit(){
        if(calibrated == 1){
            return calib_sup_limit;
        } else{
            return m_sup_limit;
        }
    };

    int CSpeedingMotor::get_lower_limit(){
        if(calibrated == 1){
            return calib_inf_limit;
        } else{
            return m_inf_limit;
        }
    };

}; // namespace hardware::drivers
