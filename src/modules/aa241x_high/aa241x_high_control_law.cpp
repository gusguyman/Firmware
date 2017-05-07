/*
 * @file aa241x_fw_control.cpp
 *
 * Implementation of basic control laws (PID) for constant yaw, roll, pitch, and altitude.
 * Also includes constant heading and constant heading + altitude functions.
 *
 *  @author Elise FOURNIER-BIDOZ		<efb@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)
float altitude_desired = 0.0f;
float altitude = 0.0f;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */

// // Make a PID yaw stabilizer // //

void constant_yaw(float dt, float yaw_desired, float previous_err, float integral) {

//    float yaw_desired = 0.0f; // yaw_desired already exists in aa241x_high_aux so no need to repeat float declaration

    float Kp = aah_parameters.proportional_yaw_gain;
    float Ki = aah_parameters.integrator_yaw_gain;
    float Kd = aah_parameters.derivative_yaw_gain;

    float err = yaw_desired - yaw;
    integral = integral + err;
    float der  = err - previous_err;
    previous_err = err;

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

void constant_roll(float dt, float roll_desired, float integral, float previous_err) {

    float Kp = aah_parameters.proportional_roll_gain;
    float Ki = aah_parameters.integrator_roll_gain;
    float Kd = aah_parameters.derivative_roll_gain;

    float err = roll_desired - roll;
    integral = integral + err;
    float der  = err - previous_err;
    previous_err = err;

//    dt = ;
    float PIDRollCorrection = Kp*err + (Ki*integral*dt) + (Kd*der/dt);

    if (PIDRollCorrection > 1.0f) {
            PIDRollCorrection = 1.0f;
    } else if (PIDRollCorrection < -1.0f ) {
            PIDRollCorrection = -1.0f;
    }


    roll_servo_out = PIDRollCorrection;

    pitch_servo_out = -man_pitch_in;
    yaw_servo_out = man_yaw_in;
    throttle_servo_out = man_throttle_in;
}

// // Make a PID pitch stabilizer // //

void constant_pitch(float dt, float pitch_desired, float previous_err, float integral) {

    float Kp = aah_parameters.proportional_pitch_gain;
    float Ki = aah_parameters.integrator_pitch_gain;
    float Kd = aah_parameters.derivative_pitch_gain;

    float err = pitch_desired - pitch;
    integral  = integral + err;
    float der  = err - previous_err;
    previous_err = err;

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

void constant_altitude(float dt, float previous_int_h, float previous_err_h, float previous_int_th, float previous_err_th, float altitude_desired) {

    float Kp_h = aah_parameters.proportional_altitude_gain;
    float Ki_h = aah_parameters.integrator_altitude_gain;
    float Kd_h = aah_parameters.derivative_altitude_gain;

    float Kp_th = aah_parameters.proportional_th_gain;
    float Ki_th = aah_parameters.integrator_th_gain;
    float Kd_th = aah_parameters.derivative_th_gain;


    // Define integral and derivative of h
    float err_h = altitude_desired - position_D_baro; // Error in altitude
    float int_h  = previous_int_h + err_h; //
    previous_int_h = int_h;

    float der  = err_h- previous_err_h;
    previous_err_h = err_h;

    float th_desired = Kp_h*err_h + (Ki_h*int_h*dt) + (Kd_h*der/dt);

    // Define integral and derivative of theta
    float err_th = th_desired - pitch;
    float int_th  = previous_int_th + err_th;
    previous_int_th = int_th;

    float der  = err_th - previous_err_th;
    previous_err_th = err_th;

    pitch_servo_out = Kp_th*err_th + (Ki_th*int_th*dt) + (Kd_th*der_th/dt)

    roll_servo_out = man_roll_in;
    yaw_servo_out = man_yaw_in;
    throttle_servo_out = man_throttle_in;


}

// // Make a heading stabilizer // //

void constant_heading(float dt, float yaw_desired, float roll_desired) {
    constant_yaw(dt, yaw_desired);
    constant_roll(dt, roll_desired);
}

// // Make an altitude + heading stabilizer // //

void constant_heading_altitude(float dt, float yaw_desired, float roll_desired, float altitude_desired) {
    constant_heading(dt, yaw_desired, roll_desired);
    constant_altitude(dt, altitude_desired);
}


//void flight_control() {
//
//        //CALL THE FUNCTION WE WANT TO EXECUTE, OR DO SEVERAL IF STATEMENTS ACCORDING
//        // TO WHAT FLIGHT MODE WE'RE IN
//
//}
