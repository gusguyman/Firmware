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
                         const in_state_s & in_alt \
                         );

        principal_axis_c& GetYaw() {return _Yaw;}
        principal_axis_c& GetPitch() {return _Pitch;}
        principal_axis_c& GetRoll() {return _Roll;}
        principal_axis_c& GetThrottle() {return _Throttle;}

        float Find_perp_distance(in_state_s in_ground_course);
        void SetPos(float in_cur_N, float in_cur_E);


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
};

#endif // MAZCONTROLLER_H
