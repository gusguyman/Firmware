/*
 * @file aa241x_fw_control_params.c
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */

#include "aa241x_high_params.h"



/*
 *  controller parameters, use max. 15 characters for param name!
 *
 */

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be C_EXAMPLE and will be in the C dropdown.  Make sure to always
 * start your parameters with C to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit meter 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be C_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with C to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */

// TODO: define custom parameters here

// Yaw gains
PARAM_DEFINE_FLOAT(AAH_YAW_P, 1.0f);
PARAM_DEFINE_FLOAT(AAH_YAW_I, 1.0f);
PARAM_DEFINE_FLOAT(AAH_YAW_D, 1.0f);


// Roll gains
PARAM_DEFINE_FLOAT(AAH_ROLL_P, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ROLL_I, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ROLL_D, 1.0f);

// Pith gains
PARAM_DEFINE_FLOAT(AAH_PITCH_P, 1.0f);
PARAM_DEFINE_FLOAT(AAH_PITCH_I, 1.0f);
PARAM_DEFINE_FLOAT(AAH_PITCH_D, 1.0f);

// Altitude gains
PARAM_DEFINE_FLOAT(AAH_ALT_P, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ALT_I, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ALT_D, 1.0f);

PARAM_DEFINE_FLOAT(AAH_HEAD_P, 1.0f);
PARAM_DEFINE_FLOAT(AAH_HEAD_I, 1.0f);
PARAM_DEFINE_FLOAT(AAH_HEAD_D, 1.0f);

// Theta gains
PARAM_DEFINE_FLOAT(AAH_TH_P, 1.0f);
PARAM_DEFINE_FLOAT(AAH_TH_I, 1.0f);
PARAM_DEFINE_FLOAT(AAH_TH_D, 1.0f);

PARAM_DEFINE_INT32(AAH_FLIGHT_MODE, 0);

PARAM_DEFINE_FLOAT(AAH_ALT_DESIRED, 50.0f);

PARAM_DEFINE_INT32(AAH_AUTOPILOT, 0);
//PARAM_DEFINE_FLOAT(AAH_VEL_DESIRED, 1.0f);

PARAM_DEFINE_FLOAT(AAH_VEL_P, 1.0f);
PARAM_DEFINE_FLOAT(AAH_VEL_I, 1.0f);
PARAM_DEFINE_FLOAT(AAH_VEL_D, 1.0f);

PARAM_DEFINE_FLOAT(AAH_ROLL_HEA_P, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ROLL_HEA_I, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ROLL_HEA_D, 1.0f);

int aah_parameters_init(struct aah_param_handles *h)
{

	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
	h-> proportional_yaw_gain 		= param_find("AAH_YAW_P");
	h-> integrator_yaw_gain 		= param_find("AAH_YAW_I");
	h-> derivative_yaw_gain 		= param_find("AAH_YAW_D");

	h-> proportional_roll_gain 		= param_find("AAH_ROLL_P");
	h-> integrator_roll_gain 		= param_find("AAH_ROLL_I");
	h-> derivative_roll_gain 		= param_find("AAH_ROLL_D");

	h-> proportional_pitch_gain		= param_find("AAH_PITCH_P");
	h-> integrator_pitch_gain 		= param_find("AAH_PITCH_I");
	h-> derivative_pitch_gain 		= param_find("AAH_PITCH_D");

	h-> proportional_altitude_gain 	= param_find("AAH_ALT_P");
	h-> integrator_altitude_gain 	= param_find("AAH_ALT_I");
	h-> derivative_altitude_gain 	= param_find("AAH_ALT_D");

    h-> proportional_heading_gain 	= param_find("AAH_HEAD_P");
	h-> integrator_heading_gain 	= param_find("AAH_HEAD_I");
	h-> derivative_heading_gain 	= param_find("AAH_HEAD_D");

	h-> proportional_th_gain 		= param_find("AAH_TH_P");
	h-> integrator_th_gain 			= param_find("AAH_TH_I");
	h-> derivative_th_gain 			= param_find("AAH_TH_D");

	h-> flight_mode 				= param_find("AAH_FLIGHT_MODE");

	h-> autopilot				= param_find("AAH_AUTOPILOT");

	h-> altitude_desired 			= param_find("AAH_ALT_DESIRED");

	h-> proportional_velocity_gain 		= param_find("AAH_VEL_P");
	h-> integrator_velocity_gain 		= param_find("AAH_VEL_I");
	h-> derivative_velocity_gain 		= param_find("AAH_VEL_D");

	h-> proportional_rollForHeading_gain 		= param_find("AAH_ROLL_HEA_P");
	h-> integrator_rollForHeading_gain 		= param_find("AAH_ROLL_HEA_I");
	h-> derivative_rollForHeading_gain 		= param_find("AAH_ROLL_HEA_D");


	// TODO: add the above line for each of your custom parameters........

	return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{

	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
	param_get(h->proportional_yaw_gain, &(p->proportional_yaw_gain));
	param_get(h->integrator_yaw_gain, &(p->integrator_yaw_gain));
	param_get(h->derivative_yaw_gain, &(p->derivative_yaw_gain));

	param_get(h->proportional_roll_gain, &(p->proportional_roll_gain));
	param_get(h->integrator_roll_gain, &(p->integrator_roll_gain));
	param_get(h->derivative_roll_gain, &(p->derivative_roll_gain));

	param_get(h->proportional_pitch_gain, &(p->proportional_pitch_gain));
	param_get(h->integrator_pitch_gain, &(p->integrator_pitch_gain));
	param_get(h->derivative_pitch_gain, &(p->derivative_pitch_gain));

	param_get(h->proportional_altitude_gain, &(p->proportional_altitude_gain));
	param_get(h->integrator_altitude_gain, &(p->integrator_altitude_gain));
	param_get(h->derivative_altitude_gain, &(p->derivative_altitude_gain));

    param_get(h->proportional_heading_gain, &(p->proportional_heading_gain));
	param_get(h->integrator_heading_gain, &(p->integrator_heading_gain));
	param_get(h->derivative_heading_gain, &(p->derivative_heading_gain));

	param_get(h->proportional_th_gain, &(p->proportional_th_gain));
	param_get(h->integrator_th_gain, &(p->integrator_th_gain));
	param_get(h->derivative_th_gain, &(p->derivative_th_gain));

	param_get(h->flight_mode, &(p->flight_mode));

	param_get(h->altitude_desired, &(p->altitude_desired));

	param_get(h->proportional_velocity_gain, &(p->proportional_velocity_gain));
	param_get(h->integrator_velocity_gain, &(p->integrator_velocity_gain));
	param_get(h->derivative_velocity_gain, &(p->derivative_velocity_gain));

	param_get(h->proportional_rollForHeading_gain, &(p->proportional_rollForHeading_gain));
	param_get(h->integrator_rollForHeading_gain, &(p->integrator_rollForHeading_gain));
	param_get(h->derivative_rollForHeading_gain, &(p->derivative_rollForHeading_gain));

	// TODO: add the above line for each of your custom parameters.....

	return OK;
}
