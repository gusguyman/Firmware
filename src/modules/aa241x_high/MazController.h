#ifndef MAZCONTROLLER_H
#define MAZCONTROLLER_H

#include "principal_axis_c.h"

struct output_s {
    float roll;
    float pitch;
    float yaw;
    float throttle;
};


struct in_state_s {
    float kp;
    float kd;
    float ki;
    float current;
    float desired;
};

struct logger_s {
    float field1;
    float field2;
    float field3;
    float field4;
    float field5;
    float field6;
    float field7;
    float field8;
    float field9;
    float field10;
    float field11;
    float field12;
    float field13;
    float field14;
    float field15;
    float field16;
};

class MazController
{
    public:
        /** Default constructor */
        MazController();
        /** Default destructor */
        virtual ~MazController();

        void Controller(int flight_mode, output_s & r_outputs, \
                         const in_state_s & in_roll, \
                         const in_state_s & in_pitch, \
                         const in_state_s & in_yaw, \
                         const in_state_s & in_alt, \
                         const in_state_s & in_heading \
                         );

        principal_axis_c& GetYaw() {return _Yaw;}
        principal_axis_c& GetPitch() {return _Pitch;}
        principal_axis_c& GetRoll() {return _Roll;}
        principal_axis_c& GetThrottle() {return _Throttle;}

        float Find_perp_distance(const in_state_s & in_ground_course);
        void SetPos(float in_cur_N, float in_cur_E);
        void GetLogData(logger_s & in_log);


    private:
        principal_axis_c _Yaw;
        principal_axis_c _Pitch;
        principal_axis_c _Roll;
        principal_axis_c _Throttle;
        principal_axis_c _Alt;
        principal_axis_c _Heading;
        float _prev_N;
        float _prev_E;
        float _prev_goal_N;
        float _prev_goal_E;
        float _cur_N;
        float _cur_E;
        struct logger_s _data_to_log;
};

#endif // MAZCONTROLLER_H
