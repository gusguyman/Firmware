#include "MazController.h"
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"
#include <cstdlib>
#include <cmath>


MazController::MazController()
{
    //ctor
    _prev_E = 0.0f;
    _prev_goal_N = 0.0f;
    _prev_N = 0.0f;
    _prev_goal_E = 0.0f;
    _cur_E = 0.0f;
    _cur_N = 0.0f;
    _data_to_log.field1 = 0.0f;
    _data_to_log.field2 = 0.0f;
    _data_to_log.field3 = 0.0f;
    _data_to_log.field4 = 0.0f;
    _data_to_log.field5 = 0.0f;
    _data_to_log.field6 = 0.0f;
    _data_to_log.field7 = 0.0f;
    _data_to_log.field8 = 0.0f;
    _data_to_log.field9 = 0.0f;
    _data_to_log.field10 = 0.0f;
    _data_to_log.field11 = 0.0f;
    _data_to_log.field12 = 0.0f;
    _data_to_log.field13 = 0.0f;
    _data_to_log.field14 = 0.0f;
    _data_to_log.field15 = 0.0f;
    _data_to_log.field16 = 0.0f;

}

MazController::~MazController()
{
    //dtor
}

void MazController::GetLogData(logger_s & in_log) {
    in_log.field1 =_data_to_log.field1;
    in_log.field2 =_data_to_log.field2;
    in_log.field3 =_data_to_log.field3;
    in_log.field4 =_data_to_log.field4;
    in_log.field5 =_data_to_log.field5;
    in_log.field6 =_data_to_log.field6;
    in_log.field7 =_data_to_log.field7;
    in_log.field8 =_data_to_log.field8;
    in_log.field9 =_data_to_log.field9;
    in_log.field10 =_data_to_log.field10;
    in_log.field11 =_data_to_log.field11;
    in_log.field12 =_data_to_log.field12;
    in_log.field13 =_data_to_log.field13;
    in_log.field14 =_data_to_log.field14;
    in_log.field15 =_data_to_log.field15;
}

float MazController::Find_perp_distance(const in_state_s & in_ground_course) {

    float d = sqrtf(pow((_cur_N - _prev_N),2.0f)+ pow((_cur_E - _prev_E),2.0f));
    float goal_N = _prev_goal_N + d * cosf(in_ground_course.desired);
    float goal_E = _prev_goal_E + d * sinf(in_ground_course.desired);
    float perp_d = sqrtf(pow((_cur_N - goal_N),2.0f) + pow((_cur_E - goal_E),2.0f));

    //Update next iters inputs and return
    _prev_goal_E = goal_E;
    _prev_goal_N = goal_N;
    _prev_E = _cur_E;
    _prev_N = _cur_N;
    return perp_d;
}

void MazController::SetPos(float in_cur_N, float in_cur_E) {
    _cur_E = in_cur_E;
    _cur_N = in_cur_N;
}

void MazController::Controller(int flight_mode, output_s & r_outputs, \
                         const in_state_s & in_roll, \
                         const in_state_s & in_pitch, \
                         const in_state_s & in_yaw, \
                         const in_state_s & in_vel, \
                         const in_state_s & in_alt, \
                         const in_state_s & in_heading \
                         ) {
    _data_to_log.field1 = float(flight_mode);
    switch(flight_mode)
    {
    case 0: // Constant Roll
        _Roll.SetGains(in_roll.kp, in_roll.kd, in_roll.ki);
        _Roll.SetDesired(in_roll.desired);
        _data_to_log.field2 = in_roll.desired;
        _Roll.SetCurrentValue(in_roll.current);
        _Roll.PID_Update();
        r_outputs.roll = _Roll.GetOutput();
        _data_to_log.field2 = in_roll.desired;
        break;
    case 1: // Constant Pitch
        _Pitch.SetGains(in_pitch.kp, in_pitch.kd, in_pitch.ki);
        _Pitch.SetDesired(in_pitch.desired);
        _data_to_log.field3 = in_pitch.desired;
        _Pitch.SetCurrentValue(in_pitch.current);
        _Pitch.PID_Update();
        r_outputs.pitch = _Pitch.GetOutput();
        _data_to_log.field3 = in_pitch.desired;
        break;
    case 2: // Constant Yaw
        _Yaw.SetGains(in_yaw.kp, in_yaw.kd, in_yaw.ki);
        _Yaw.SetDesired(in_yaw.desired);
        _data_to_log.field4 = in_yaw.desired
        _Yaw.SetCurrentValue(in_yaw.current);
        _Yaw.PID_Update();
        r_outputs.yaw = _Yaw.GetOutput();
        _data_to_log.field4 = in_yaw.desired;
        break;
    case 3: //const Alt
        _Roll.SetGains(in_roll.kp, in_roll.kd, in_roll.ki);
        _Roll.SetDesired(in_roll.desired);
        _Roll.SetCurrentValue(in_roll.current);
        _Roll.PID_Update();
        r_outputs.roll = _Roll.GetOutput();
        _data_to_log.field2 = in_roll.desired;

        _Vel.SetGains(in_vel.kp, in_vel.kd, in_vel.ki);
        _Vel.SetDesired(in_vel.desired);
        _Vel.SetCurrentValue(in_vel.current);
        _Vel.PID_Update();
        r_outputs.vel = _Vel.GetOutput();
        _data_to_log.field5 = in_vel.desired;

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
        _data_to_log.field3 = in_pitch.desired;

        _data_to_log.field2 = in_roll.desired;
        _data_to_log.field6 = in_alt.desired;
        _data_to_log.field7 = _Alt.GetOutput();

        break;
    case 4: //const Heading
        _Heading.SetDesired(0.0f); //Want no perp distance to line
        _Heading.SetCurrentValue(Find_perp_distance(in_heading));
        _Heading.SetGains(in_heading.kp, in_heading.kd, in_heading.ki);
        _Heading.PID_Update();

        _Yaw.SetGains(in_yaw.kp, in_yaw.kd, in_yaw.ki);
        _Yaw.SetDesired(_Heading.GetOutput() + (in_yaw.desired - in_heading.desired));
        _Yaw.SetCurrentValue(in_yaw.current);
        _Yaw.PID_Update();
        r_outputs.yaw = _Yaw.GetOutput();

        _Roll.SetGains(in_roll.kp, in_roll.kd, in_roll.ki);
        _Roll.SetDesired(in_roll.desired);
        _Roll.SetCurrentValue(in_roll.current);
        _Roll.PID_Update();
        r_outputs.roll = _Roll.GetOutput();
        _data_to_log.field2 = in_roll.desired;

        _data_to_log.field8 = _Heading.GetOutput() + (in_yaw.desired - in_heading.desired);

        _data_to_log.field10 = in_heading.desired;
        _data_to_log.field11 = _prev_goal_N;
        _data_to_log.field12 = _prev_goal_E;
        _data_to_log.field13 = _prev_N;
        _data_to_log.field14 = _prev_E;

        break;
    case 5:
        _Heading.SetDesired(0.0f); //Want no perp distance to line
        _Heading.SetCurrentValue(Find_perp_distance(in_heading));
        _Heading.SetGains(in_heading.kp, in_heading.kd, in_heading.ki);
        _Heading.PID_Update();
        _data_to_log.field8 = _Heading.GetOutput() + (in_yaw.desired - in_heading.desired);

        _data_to_log.field10 = in_heading.desired;
        _data_to_log.field11 = _prev_goal_N;
        _data_to_log.field12 = _prev_goal_E;
        _data_to_log.field13 = _prev_N;
        _data_to_log.field14 = _prev_E;

        _Yaw.SetGains(in_yaw.kp, in_yaw.kd, in_yaw.ki);
        _Yaw.SetDesired(_Heading.GetOutput() + (in_yaw.desired - in_heading.desired));
        _Yaw.SetCurrentValue(in_yaw.current);
        _Yaw.PID_Update();
        r_outputs.yaw = _Yaw.GetOutput();
        _data_to_log.field4 = in_yaw.desired;

        _Roll.SetGains(in_roll.kp, in_roll.kd, in_roll.ki);
        _Roll.SetDesired(in_roll.desired);
        _Roll.SetCurrentValue(in_roll.current);
        _Roll.PID_Update();
        r_outputs.roll = _Roll.GetOutput();
        _data_to_log.field2 = in_roll.desired;

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
        _data_to_log.field3 = in_pitch.desired;
        _data_to_log.field6 = in_alt.desired;
        _data_to_log.field7 = _Alt.GetOutput();
        break;
    case 6: //Follow a line according to Rock's method, with no altitude hold
        _Heading.SetDesired(0.0f); //Want no perp distance to line
        _Heading.SetCurrentValue(Find_perp_distance(in_heading));
        _Heading.SetGains(in_heading.kp, in_heading.kd, in_heading.ki);
        _Heading.PID_Update();
        _data_to_log.field8 = _Heading.GetOutput() + (in_yaw.desired - in_heading.desired);

        _data_to_log.field10 = in_heading.desired;
        _data_to_log.field11 = _prev_goal_N;
        _data_to_log.field12 = _prev_goal_E;
        _data_to_log.field13 = _prev_N;
        _data_to_log.field14 = _prev_E;

        _Yaw.SetGains(in_yaw.kp, in_yaw.kd, in_yaw.ki);
        _Yaw.SetDesired(_Heading.GetOutput()); //+ (in_yaw.desired - in_heading.desired));
        _Yaw.SetCurrentValue(in_yaw.current);
        _Yaw.PID_Update();
        r_outputs.yaw = _Yaw.GetOutput();
        _data_to_log.field4 = in_yaw.desired;

        _Roll.SetGains(in_roll.kp, in_roll.kd, in_roll.ki);
        _Roll.SetDesired(_Yaw.GetOutput());
        _Roll.SetCurrentValue(in_roll.current);
        _Roll.PID_Update();
        r_outputs.roll = _Roll.GetOutput();
        _data_to_log.field2 = in_roll.desired;
        break;

    case 7:
        _Vel.SetGains(in_vel.kp, in_vel.kd, in_vel.ki);
        _Vel.SetDesired(in_vel.desired);
        _Vel.SetCurrentValue(in_vel.current);
        _Vel.PID_Update();
        r_outputs.vel = _Vel.GetOutput();
        _data_to_log.field5 = in_vel.desired;

        _Heading.SetDesired(0.0f); //Want no perp distance to line
        _Heading.SetCurrentValue(Find_perp_distance(in_heading));
        _Heading.SetGains(in_heading.kp, in_heading.kd, in_heading.ki);
        _Heading.PID_Update();
        _data_to_log.field8 = _Heading.GetOutput() + (in_yaw.desired - in_heading.desired);

        _data_to_log.field10 = in_heading.desired;
        _data_to_log.field11 = _prev_goal_N;
        _data_to_log.field12 = _prev_goal_E;
        _data_to_log.field13 = _prev_N;
        _data_to_log.field14 = _prev_E;

        _Yaw.SetGains(in_yaw.kp, in_yaw.kd, in_yaw.ki);
        _Yaw.SetDesired(_Heading.GetOutput() + (in_yaw.desired - in_heading.desired));
        _Yaw.SetCurrentValue(in_yaw.current);
        _Yaw.PID_Update();
        r_outputs.yaw = _Yaw.GetOutput();
        _data_to_log.field4 = in_yaw.desired;

        _Roll.SetGains(in_roll.kp, in_roll.kd, in_roll.ki);
        _Roll.SetDesired(in_roll.desired);
        _Roll.SetCurrentValue(in_roll.current);
        _Roll.PID_Update();
        r_outputs.roll = _Roll.GetOutput();
        _data_to_log.field2 = in_roll.desired;

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
        _data_to_log.field3 = in_pitch.desired;
        _data_to_log.field6 = in_alt.desired;
        _data_to_log.field7 = _Alt.GetOutput();

    default:
        //Do nothing, return all manual inputs
        break;
    }
}
