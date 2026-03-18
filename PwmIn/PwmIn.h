#ifndef MBED_PWMIN_H
#define MBED_PWMIN_H

#include <mbed.h>
#include <chrono>

using namespace std::chrono;

class PwmIn
{
    public:
        PwmIn(PinName p);

        float period();
        float pulsewidth();
        float dutycycle();

    private:
        void rise_handler();
        void fall_handler();

        InterruptIn _input;
        Timer _timer;
        float _pulsewidth;
        float _period;
};

#endif // MBED_PWMIN_H
