/*
 * @file aa241x_fw_control.cpp
 *
 * Implementation of basic control laws (PID) for constant yaw, roll, pitch, and altitude.
 * Also includes constant heading and constant heading + altitude functions.
 *
 *  @author Elise FOURNIER-BIDOZ		<efb@stanford.edu>
 *  @author Alexandre EL ASSAD          <aelassad@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)

//SHOULD WE DEFINE THESE FOR ROLL-FOR-HEADING ?
float previous_err_yaw = 0.0f;
float previous_integral_yaw = 0.0f;
float previous_err_pitch = 0.0f;
float previous_integral_pitch = 0.0f;
float previous_err_roll = 0.0f;
float previous_integral_roll = 0.0f;
float previous_err_h = 0.0f;
float previous_integral_h = 0.0f;
float previous_err_th = 0.0f;
float previous_integral_th = 0.0f;
float dt = 1.0/60;

int flight_mode = aah_parameters.flight_mode;
int autopilot = aah_parameters.autopilot;
float velocity_desired = 0.0f;
float altitude_desired = 0.0f;
float grndspeed_desired = 0.0f;
float heading_desired = 0.0f;

int current_command = 0;

const float PI = 3.1415927;


/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */
in_state_s roll_s;
in_state_s pitch_s;
in_state_s yaw_s;
in_state_s vel_s;
in_state_s alt_s;
in_state_s heading_s;
in_state_s rollForHeading_s;

logger_s data_to_log;
std::vector<float> lat_vals(3);
std::vector<float> lon_vals(3);
int target_idx = 0;
float target_yaw = 0.0f;
bool new_targets = false;
std::vector<target_s> target_list;
void low_loop();

bool first_run = true;

bool autopilot_is_off(){
    return (aah_parameters.autopilot == 0);
}

bool turn_is_complete(float ep) {
    float diff = std::fabs(yaw - target_yaw);
    return (diff < ep);

}

bool straight_is_complete(float ep) {
    float dist = sqrtf(pow(position_N - target_list[target_idx].pos_N, 2) + \
                      pow(position_E - target_list[target_idx].pos_E,2));
    return (dist < ep);
}

bool turning() {
    return (current_command == 0);
}

bool going_straight() {
    return (current_command == 1);
}

void UpdateInputs(in_state_s & in_roll, \
                  in_state_s & in_pitch, \
                  in_state_s & in_yaw, \
                  in_state_s & in_vel, \
                  in_state_s & in_alt, \
                  in_state_s & in_heading, \
                  in_state_s & in_rollForHeading \
                  ) {
//Update Parameter stuff

    in_roll.kp = aah_parameters.proportional_roll_gain;
    in_roll.kd = aah_parameters.derivative_roll_gain;
    in_roll.ki = aah_parameters.integrator_roll_gain;
    in_roll.current = roll;
    in_roll.desired = roll_desired;

    in_pitch.kp = aah_parameters.proportional_pitch_gain;
    in_pitch.kd = aah_parameters.derivative_pitch_gain;
    in_pitch.ki = aah_parameters.integrator_pitch_gain;
    in_pitch.current = pitch;
    in_pitch.desired = pitch_desired;

    in_yaw.kp = aah_parameters.proportional_yaw_gain;
    in_yaw.kd = aah_parameters.derivative_yaw_gain;
    in_yaw.ki = aah_parameters.integrator_yaw_gain;
    in_yaw.current = yaw;
    in_yaw.desired = yaw_desired;

    in_vel.kp = aah_parameters.proportional_velocity_gain;
    in_vel.kd = aah_parameters.derivative_velocity_gain;
    in_vel.ki = aah_parameters.integrator_velocity_gain;
    in_vel.current = speed_body_u;
    in_vel.desired = velocity_desired;

    in_alt.kp = aah_parameters.proportional_altitude_gain;
    in_alt.kd = aah_parameters.derivative_altitude_gain;
    in_alt.ki = aah_parameters.integrator_altitude_gain;
    in_alt.current = position_D_gps;
    in_alt.desired = altitude_desired;

    in_heading.kp = aah_parameters.proportional_heading_gain;
    in_heading.kd = aah_parameters.derivative_heading_gain;
    in_heading.ki = aah_parameters.integrator_heading_gain;
    in_heading.current = ground_course;
    in_heading.desired = heading_desired;

    in_rollForHeading.kp = aah_parameters.proportional_rollForHeading_gain;
    in_rollForHeading.kd = aah_parameters.derivative_rollForHeading_gain;
    in_rollForHeading.ki = aah_parameters.integrator_rollForHeading_gain;
    in_rollForHeading.current = roll;
    in_rollForHeading.desired = rollForHeading_desired;
}

MazController mazController;
//mazController.GetLogData(data_to_log);
bool use_targets = true;
void flight_control() {
	if (first_run) {
	    target_list.reserve(5);
	    target_list.clear();
	    target_s target0;
	    target_s target1;
	    target_s target2;
	    target_s target3;

	    target0.yaw = 0.0f;
	    target0.turnLeft = true;
        target0.pos_E = position_E;
        target0.pos_N = position_N;
        target0.radius = 10.0f;
	    target_list.push_back(target0);

	    //target1.yaw = 0.5f;
	    //target1.pos_E = 1944.0f; //Known point in Coyote Hill
	    //target1.pos_N = -2400.0f; //Known point in Coyote Hill
        target1.yaw = 0.0f;
	    target1.turnLeft = true;
        target1.pos_E = position_E;
        target1.pos_N = position_N + 50.0f;
        target1.radius = 10.0f;
	    target_list.push_back(target1);

	    //target2.yaw = 0.0f;
	    //target2.pos_E = 1944.0f; //Known point in Coyote Hill
	    //target2.pos_N = -2200.0f; //Known point in Coyote Hill
        target2.yaw = 0.57f;
	    target2.turnLeft = true;
        target2.pos_E = position_E - 50.0f;
        target2.pos_N = position_N + 50.0f;
        target2.radius = 10.0f;
	    target_list.push_back(target2);

	    //target3.yaw = 0.785;
	    //target3.pos_E = 1800.0f; //Known point in Coyote Hill
	    //target3.pos_N = -2200.0f; //Known point in Coyote Hill
        target3.yaw = -2.5f;
	    target3.turnLeft = true;
        target3.pos_E = position_E;
        target3.pos_N = position_N;
        target3.radius = 1.5f;
	    target_list.push_back(target3);

	    first_run = false;
	    new_targets = true;
	}
    if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
                                                                 //	should only occur on first engagement since this is 59Hz loop
        yaw_desired = yaw;
        roll_desired = roll;
        pitch_desired = pitch;
        velocity_desired = speed_body_u;
        heading_desired = ground_course;
        altitude_desired = aah_parameters.altitude_desired;
        rollForHeading_desired = roll;
        mazController.SetPosInit(position_N, position_E);
        first_run = true;
         							// yaw_desired already defined in aa241x_high_aux.h
//    altitude_desired = position_D_baro; 		// altitude_desired needs to be declared outside flight_control() function
    }

	//Set Default Outputs to manual;
    output_s outputs;
    outputs.pitch = man_pitch_in;
    outputs.roll = man_roll_in;
    outputs.yaw = man_yaw_in;
    outputs.throttle = man_throttle_in;
    outputs.rollForHeading = man_roll_in;

    if(new_targets) {
        target_idx = 1;
        new_targets = false;
        current_command = 0;
        flight_mode = target_list[target_idx].turnLeft ? \
            mazController.turn_left() : \
            mazController.turn_right();
        target_yaw = target_list[target_idx].yaw;

    } else {
        if (turning()) {
            if (turn_is_complete(0.2f)) {
                mazController.SetPosInit(position_N, position_E);
                mazController.SetGoal(target_list[target_idx].pos_N, target_list[target_idx].pos_E);
                current_command = 1;
                mazController.SetYaw(target_list[target_idx].pos_E, target_list[target_idx].pos_N, position_E, position_N);
                flight_mode = mazController.follow_line();
                //SET desired target if needed
            } else {
                // nothing to be done here
            }
        } else {
            if (straight_is_complete(target_list[target_idx].radius)) {
                target_idx ++;
                if (target_idx > target_list.size()) { // Out of targets, hold course until new targets
                    yaw_desired = yaw;
                    roll_desired = 0;
                    pitch_desired = 0;
                    velocity_desired = 15.0f;
                    altitude_desired = position_D_gps;
                    current_command = 1;
                } else {
                    flight_mode = target_list[target_idx].turnLeft ? \
                        mazController.turn_left() : \
                        mazController.turn_right();
                    target_yaw = target_list[target_idx].yaw;
                    current_command = 0;
                }
            } else {
                mazController.SetPos(position_N, position_E);
                //Test if we missed target
                if ( (position_N - target_list[target_idx].pos_N)*(target_list[target_idx].pos_N - target_list[target_idx-1].pos_N) + \
                     (position_E - target_list[target_idx].pos_E)*(target_list[target_idx].pos_E - target_list[target_idx-1].pos_E)  > 0.0f ) { //we missed target, skip it
                	target_idx ++;
                	if (target_idx > target_list.size()) { // Out of targets, hold course until new targets
                    	yaw_desired = yaw;
                    	roll_desired = 0;
                    	pitch_desired = 0;
                    	velocity_desired = 15.0f;
                    	altitude_desired = position_D_gps;
                    	current_command = 1;
                    } else {
                    	flight_mode = target_list[target_idx].turnLeft ? \
                        	mazController.turn_left() : \
                        	mazController.turn_right();
                    	target_yaw = target_list[target_idx].yaw;
                    	current_command = 0;
                    }
                }
            }
        }
    }
    if (autopilot_is_off()) {
        flight_mode = aah_parameters.flight_mode;
    }

    UpdateInputs(roll_s, pitch_s, yaw_s, vel_s, alt_s, heading_s, rollForHeading_s);

    mazController.Controller(flight_mode, outputs, \
                             roll_s, \
                             pitch_s, \
                             yaw_s, \
                             vel_s, \
                             alt_s, \
                             heading_s,\
                             rollForHeading_s \
                             );

    mazController.GetLogData(data_to_log);
    data_to_log.field5 = speed_body_u;
    data_to_log.field16 = target_idx;
    yaw_servo_out = outputs.yaw;
    pitch_servo_out = -outputs.pitch; // Negative for preferred control inversion
    roll_servo_out = outputs.roll;
    throttle_servo_out = outputs.throttle;

}
