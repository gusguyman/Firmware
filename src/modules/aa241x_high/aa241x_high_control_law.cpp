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

// // Make a PID yaw stabilizer // //

void constant_yaw() {

//    float yaw_desired = 0.0f; // yaw_desired already exists in aa241x_high_aux so no need to repeat float declaration

    float Kp = aah_parameters.proportional_yaw_gain;
    float Ki = aah_parameters.integrator_yaw_gain;
    float Kd = aah_parameters.derivative_yaw_gain;

    float err = yaw_desired - yaw;
    float integral= previous_integral_yaw + err;
    float der  = err - previous_err_yaw;
    previous_err_yaw = err;
    previous_integral_yaw = integral;

//    float dt = 1.0 / 60.0;//execution time of loop.
    float PIDYawCorrection = Kp*err + (Ki*integral*dt) + (Kd*der/dt);

    // Do bounds checking to keep the yaw correction within the -1..1 limits of the servo output
    if (PIDYawCorrection > 1.0f) {
            PIDYawCorrection = 1.0f;
    } else if (PIDYawCorrection < -1.0f ) {
            PIDYawCorrection = -1.0f;
    }

    // ENSURE THAT YOU SET THE SERVO OUTPUTS!!!
    // outputs should be set to values between -1..1 (except throttle is 0..1)
    // where zero is no actuation, and -1,1 are full throw in either the + or - directions

    // Set output of roll servo to the control law output calculated above
    yaw_servo_out = PIDYawCorrection;
    // as an example, just passing through manual control to everything but roll
    pitch_servo_out = -man_pitch_in;
    roll_servo_out = man_roll_in;
    throttle_servo_out = man_throttle_in;
}

// // Make a PID roll stabilizer // //

void constant_roll() {

    float Kp = aah_parameters.proportional_roll_gain;
    float Ki = aah_parameters.integrator_roll_gain;
    float Kd = aah_parameters.derivative_roll_gain;

    float err = roll_desired - roll;
    float integral = previous_integral_roll + err;
    float der  = err - previous_err_roll;
    previous_err_roll = err;
    previous_integral_roll = integral;

//    dt = ;
    float PIDRollCorrection = Kp*err + (Ki*integral*dt) + (Kd*der/dt);

    if (PIDRollCorrection > 1.0f) {
            PIDRollCorrection = 1.0f;
    } else if (PIDRollCorrection < -1.0f ) {
            PIDRollCorrection = -1.0f;
    }


    roll_servo_out = -PIDRollCorrection;

    pitch_servo_out = -man_pitch_in;
    yaw_servo_out = man_yaw_in;
    throttle_servo_out = man_throttle_in;
}

// // Make a PID pitch stabilizer // //

void constant_pitch() {

    float Kp = aah_parameters.proportional_pitch_gain;
    float Ki = aah_parameters.integrator_pitch_gain;
    float Kd = aah_parameters.derivative_pitch_gain;

    float err = pitch_desired - pitch;
    float integral  = previous_integral_pitch + err;
    float der  = err - previous_err_pitch;
    previous_err_pitch = err;
    previous_integral_pitch = integral;

    float PIDPitchCorrection = Kp*err + (Ki*integral*dt) + (Kd*der/dt);

    if (PIDPitchCorrection > 1.0f) {
            PIDPitchCorrection = 1.0f;
    } else if (PIDPitchCorrection < -1.0f ) {
            PIDPitchCorrection = -1.0f;
    }


    pitch_servo_out = PIDPitchCorrection;
    roll_servo_out = man_roll_in;
    yaw_servo_out = man_yaw_in;
    throttle_servo_out = man_throttle_in;
}


// Constant altitude control law
// Based on 271A - HW3


// Altitude desired has to be defined outside the control law

void constant_altitude() {

    float Kp_h = aah_parameters.proportional_altitude_gain;
    float Ki_h = aah_parameters.integrator_altitude_gain;
    float Kd_h = aah_parameters.derivative_altitude_gain;

    float Kp_th = aah_parameters.proportional_th_gain;
    float Ki_th = aah_parameters.integrator_th_gain;
    float Kd_th = aah_parameters.derivative_th_gain;


    // Define integral and derivative of h
    float err_h = altitude_desired - position_D_baro; // Error in altitude
    float int_h  = previous_integral_h + err_h; //
    previous_integral_h = int_h;

    float der_h  = err_h- previous_err_h;
    previous_err_h = err_h;


    th_desired = Kp_h*err_h + (Ki_h*int_h*dt) + (Kd_h*der_h/dt);
    if (th_desired > 0.785f){
        th_desired = 0.785f;
    } else if (th_desired < -0.785f){
        th_desired = -0.785f;
    }

    // Define integral and derivative of theta
    float err_th = th_desired - pitch;
    float int_th  = previous_integral_th + err_th;
    previous_integral_th = int_th;

    float der_th  = err_th - previous_err_th;
    previous_err_th = err_th;

    float pitch_correction = Kp_th*err_th + (Ki_th*int_th*dt) + (Kd_th*der_th/dt);

    if (pitch_correction > 1.0f){
        pitch_correction = 1.0f;
    } else if (pitch_correction < -1.0f){
        pitch_correction = -1.0f;
    }

    pitch_servo_out = -pitch_correction;

    roll_servo_out = man_roll_in;
    yaw_servo_out = man_yaw_in;
    throttle_servo_out = man_throttle_in;


}

// // Make a heading stabilizer // //

void constant_heading() {
    constant_yaw();
    constant_roll();
}

// // Make an altitude + heading stabilizer // //

void constant_heading_altitude() {
    constant_heading();
    constant_altitude();
}


void flight_control() {

	constant_altitude();

}
