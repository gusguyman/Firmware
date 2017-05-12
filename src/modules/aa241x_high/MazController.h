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


    private:
        principal_axis_c _Yaw;
        principal_axis_c _Pitch;
        principal_axis_c _Roll;
        principal_axis_c _Throttle;
        principal_axis_c _Alt;
};

#endif // MAZCONTROLLER_H
