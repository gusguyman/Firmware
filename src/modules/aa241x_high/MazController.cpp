#include "MazController.h"
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"
#include <cstdlib>


MazController::MazController()
{
    //ctor
}

MazController::~MazController()
{
    //dtor
}


void MazController::Controller(int flight_mode, output_s & r_outputs, \
                         const in_state_s & in_roll, \
                         const in_state_s & in_pitch, \
                         const in_state_s & in_yaw, \
                         const in_state_s & in_alt \
                         ) {

    switch(flight_mode)
    {
    case 0: // Constant Roll
        _Roll.SetGains(in_roll.kp, in_roll.kd, in_roll.ki);
        _Roll.SetDesired(in_roll.desired);
        _Roll.SetCurrentValue(in_roll.current);
        _Roll.PID_Update();
        r_outputs.roll = _Roll.GetOutput();
        break;
    case 1: // Constant Pitch
        _Pitch.SetGains(in_pitch.kp, in_pitch.kd, in_pitch.ki);
        _Pitch.SetDesired(in_pitch.desired);
        _Pitch.SetCurrentValue(in_pitch.current);
        _Pitch.PID_Update();
        r_outputs.pitch = _Pitch.GetOutput();
        break;
    case 2: // Constant Yaw
        _Yaw.SetGains(in_yaw.kp, in_yaw.kd, in_yaw.ki);
        _Yaw.SetDesired(in_yaw.desired);
        _Yaw.SetCurrentValue(in_yaw.current);
        _Yaw.PID_Update();
        r_outputs.yaw = _Yaw.GetOutput();
        break;
    case 3: //const Alt
        _Roll.SetGains(in_roll.kp, in_roll.kd, in_roll.ki);
        _Roll.SetDesired(in_roll.desired);
        _Roll.SetCurrentValue(in_roll.current);
        _Roll.PID_Update();
        r_outputs.roll = _Roll.GetOutput();
        _Alt.SetGains(in_alt.kp, in_alt.kd, in_alt.ki);
        _Alt.SetBounds(-0.5f, 0.5f);
        float current;
        if (abs(in_alt.desired - in_alt.current) < 1.00) {
            current = in_alt.desired;
        } else {
            current = in_alt.current;
        }
        _Alt.SetDesired(in_alt.desired);
        _Alt.SetCurrentValue(current);
        _Alt.PID_Update();
        _Pitch.SetGains(in_pitch.kp, in_pitch.kd, in_pitch.ki);
        _Pitch.SetDesired(_Alt.GetOutput());
        _Pitch.SetCurrentValue(in_pitch.current);
        _Pitch.PID_Update();
        r_outputs.pitch = _Pitch.GetOutput();
        break;
    case 4: //const Heading
        _Roll.SetGains(in_roll.kp, in_roll.kd, in_roll.ki);
        _Roll.SetDesired(in_roll.desired);
        _Roll.SetCurrentValue(in_roll.current);
        _Roll.PID_Update();
        r_outputs.roll = _Roll.GetOutput();

        _Yaw.SetGains(in_yaw.kp, in_yaw.kd, in_yaw.ki);
        _Yaw.SetDesired(in_yaw.desired);
        _Yaw.SetCurrentValue(in_yaw.current);
        _Yaw.PID_Update();
        r_outputs.yaw = _Yaw.GetOutput();
        break;

    default:
        //Do nothing, return all manual inputs
        break;
    }
}

