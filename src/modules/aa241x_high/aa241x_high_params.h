/*
 * @file aa241x_fw_control_params.h
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Elise Fournier-Bidoz  <efbp@stanford.edu>/home/aa274/aa241x_firmware/Firmware/src/modules/aa241x_high/home/aa274/aa241x_firmware/Firmware/src/modules/aa241x_high
 */
#pragma once

#ifndef AA241X_FW_CONTROL_PARAMS_H_
#define AA241X_FW_CONTROL_PARAMS_H_


#include <systemlib/param/param.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Struct of all of the custom parameters.
 *
 * Please make sure to add a variable for each of your newly defined
 * parameters here.
 */
struct aah_params {

	// PID Gains

        float proportional_yaw_gain;
        float integrator_yaw_gain;
        float derivative_yaw_gain;

        float proportional_roll_gain;
        float integrator_roll_gain;
        float derivative_roll_gain;

        float proportional_pitch_gain;
        float integrator_pitch_gain;
        float derivative_pitch_gain;

        float proportional_altitude_gain;
        float integrator_altitude_gain;
        float derivative_altitude_gain;

        float proportional_th_gain;
        float integrator_th_gain;
        float derivative_th_gain;

        float proportional_velocity_gain;
        float integrator_velocity_gain;
        float derivative_velocity_gain;

        // Variable to define which flight mode we want:
        // 0: constant altitude
        // 1: constant heading
        // 2: constant pitch
        // 3: constant roll
        // 4: constant yaw

        int flight_mode;

        // Altitude desired
        float altitude_desired;

	// TODO: add custom parameter variable names here......

};


/**
 * Struct of handles to all of the custom parameters.
 *
 *  Please make sure to add a variable for each of your newly
 *  defined parameters here.
 *
 *  NOTE: these variable names can be the same as the ones above
 *  (makes life easier if they are)
 */
struct aah_param_handles {

        param_t proportional_yaw_gain;
        param_t integrator_yaw_gain;
        param_t derivative_yaw_gain;

        param_t proportional_roll_gain;
        param_t integrator_roll_gain;
        param_t derivative_roll_gain;

        param_t proportional_pitch_gain;
        param_t integrator_pitch_gain;
        param_t derivative_pitch_gain;

        param_t proportional_altitude_gain;
        param_t integrator_altitude_gain;
        param_t derivative_altitude_gain;

        param_t proportional_th_gain;
        param_t integrator_th_gain;
        param_t derivative_th_gain;

        param_t proportional_velocity_gain;
        param_t integrator_velocity_gain;
        param_t derivative_velocity_gain;

        param_t flight_mode;

        param_t altitude_desired;

	// TODO: add custom parameter variable names here.......

};

/**
 * Initialize all parameter handles and values
 *
 */
int aah_parameters_init(struct aah_param_handles *h);

/**
 * Update all parameters
 *
 */


int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p);

#ifdef __cplusplus
}
#endif




#endif /* AA241X_FW_CONTROL_PARAMS_H_ */
