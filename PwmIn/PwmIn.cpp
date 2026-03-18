#include "PwmIn.h"

PwmIn::PwmIn(PinName p)
    : _input(p)
    , _pulsewidth(0.0f)
    , _period(0.0f)
{
    _input.rise(callback(this, &PwmIn::rise_handler));
    _input.fall(callback(this, &PwmIn::fall_handler));

    _timer.start();
}

float PwmIn::period()
{
    return _period;
}

float PwmIn::pulsewidth()
{
    return _pulsewidth;
}

float PwmIn::dutycycle()
{
    return (_period > 0.0f) ? (_pulsewidth / _period) : 0.0f;
}

void PwmIn::rise_handler()
{
    _period = std::chrono::duration<float>(_timer.elapsed_time()).count();
    _timer.reset();
}

void PwmIn::fall_handler()
{
    _pulsewidth = std::chrono::duration<float>(_timer.elapsed_time()).count();
}
