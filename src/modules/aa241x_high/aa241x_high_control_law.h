/*
 * @file aa241x_fw_control.h
 *
 * Header file for student's fixedwing control law.
 *
 *  @author Elise FOURNIER-BIDOZ		<efb@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */
#pragma once

#ifndef AA241X_FW_CONTROL_MAIN_H_
#define AA241X_FW_CONTROL_MAIN_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include "aa241x_high_aux.h"
#include "../aa241x_low/aa241x_low_control_law.h"
#include "MazController.h"


// TODO: write your function prototypes here

void constant_yaw();

void constant_roll();

void constant_pitch();

void constant_altitude();

void constant_heading();

void constant_heading_altitude();

void UpdateInputs(in_state_s & in_roll, \
                  in_state_s & in_pitch, \
                  in_state_s & in_yaw, \
                  in_state_s & in_vel, \
                  in_state_s & in_alt, \
                  in_state_s & in_heading, \
                  in_state_s & in_rollForHeading \
                  );

bool turn_is_complete(float ep);
bool straight_is_complete(float ep) ;
bool turning() ;
bool going_straight() ;

void low_loop();

bool autopilot_is_off();

struct target_s {
    float lat;
    float pos_N;
    float lon;
    float pos_E;
    float radius;
    bool turnLeft;
    float yaw;
};

extern std::vector<target_s> target_list;
//target_list.reserve(5);
extern bool new_targets;
extern bool first_run;
extern int target_idx;
extern float target_yaw;

#endif /* AA241X_FW_CONTROL_MAIN_H_ */
