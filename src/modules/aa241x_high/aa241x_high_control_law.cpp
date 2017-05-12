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
float altitude_desired = aah_parameters.altitude_desired;

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
in_state_s alt_s;

void UpdateInputs(in_state_s & in_roll, \
                  in_state_s & in_pitch, \
                  in_state_s & in_yaw, \
                  in_state_s & in_alt \
                  ) {

    in_roll.kp = aah_parameters.proportional_roll_gain;
    in_roll.kd = aah_parameters.derivative_roll_gain;
    in_roll.ki = aah_parameters.integrator_roll_gain;
    in_roll.current = roll;
    in_roll.desired = 0.0f;

    in_pitch.kp = aah_parameters.proportional_pitch_gain;
    in_pitch.kd = aah_parameters.derivative_pitch_gain;
    in_pitch.ki = aah_parameters.integrator_pitch_gain;
    in_pitch.current = pitch;
    in_pitch.desired = 0.0f;

    in_yaw.kp = aah_parameters.proportional_yaw_gain;
    in_yaw.kd = aah_parameters.derivative_yaw_gain;
    in_yaw.ki = aah_parameters.integrator_yaw_gain;
    in_yaw.current = yaw;
    in_yaw.desired = 0.0f;

    in_alt.kp = aah_parameters.proportional_altitude_gain;
    in_alt.kd = aah_parameters.derivative_altitude_gain;
    in_alt.ki = aah_parameters.integrator_altitude_gain;
    in_alt.current = position_D_baro;
    in_alt.desired = altitude_desired;
}

MazController mazController;

void flight_control() {
    if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
                                                                 //	should only occur on first engagement since this is 59Hz loop
//    yaw_desired = yaw; 							// yaw_desired already defined in aa241x_high_aux.h
//    altitude_desired = position_D_baro; 		// altitude_desired needs to be declared outside flight_control() function
	}

	//Set Default Outputs to manual;
	output_s outputs;
	outputs.pitch = man_pitch_in;
	outputs.roll = man_roll_in;
	outputs.yaw = man_yaw_in;
	outputs.throttle = man_throttle_in;

	UpdateInputs(roll_s, pitch_s, yaw_s, alt_s);

    mazController.Controller(flight_mode, outputs, \
                             roll_s, \
                             pitch_s, \
                             yaw_s, \
                             alt_s \
                             );

    yaw_servo_out = outputs.yaw;
    pitch_servo_out = -outputs.pitch; // Negative for preferred control inversion
    roll_servo_out = outputs.roll;
    throttle_servo_out = outputs.throttle;

}
