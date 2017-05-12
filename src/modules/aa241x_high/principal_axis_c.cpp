#include "principal_axis_c.h"
#include <algorithm>

principal_axis_c::principal_axis_c()
{
    _max                = 1.0f;
    _min                = -1.0f;
    _desired            = 0.0f;
    _output             = 0.0f;
    _dt                 = 1.0f;

    _current.derr       = 0.0f;
    _current.err        = 0.0f;
    _current.integral   = 0.0f;
    _current.value      = 0.0f;

    _prev.err           = 0.0f;
    _prev.integral      = 0.0f;

    _gains.kd           = 0.0f;
    _gains.ki           = 0.0f;
    _gains.kp           = 0.0f;
}

principal_axis_c::~principal_axis_c()
{
    //dtor
}

void principal_axis_c::SetGains(float in_kp, float in_kd, float in_ki) {
    _gains.kd = in_kd;
    _gains.ki = in_ki;
    _gains.kp = in_kp;

}

void principal_axis_c::SetDesired(float in_desired) {
    _desired = in_desired;
}

void principal_axis_c::SetCurrentValue(float in_value) {
    _current.value = in_value;
}

void principal_axis_c::SetBounds(float in_min, float in_max)
{
    _min = in_min;
    _max = in_max;
}


void principal_axis_c::PID_Update() {
    _current.err = _desired - _current.value;
    _current.integral = _prev.integral + _current.err;
    _current.derr = _current.err - _prev.err;

    _output = std::min(_max, std::max(_min, \
                                      _gains.kp * _current.err + \
                                      _gains.ki * _current.integral * _dt + \
                                      _gains.kd * _current.derr / _dt));

    _prev.err = _current.err;
    _prev.integral = _current.integral;
}

