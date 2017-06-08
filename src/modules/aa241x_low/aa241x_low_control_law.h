/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file aa241x_low.h
 *
 * Header file for student's low priority control law.
 * Runs at ~ XX HZ TODO: figure out XX
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */
#ifndef AA241X_LOW_H_
#define AA241X_LOW_H_

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
#include <vector>
#include <numeric>
#include <algorithm>
//#include <iterator>
//#include <iostream>
#include <cmath>
//#include <iterator>
/*
 * Declare variables here that you may want to access
 * in multiple different function.
 */

/*
 * Declare function prototypes here.
 */


// struct target_s {
//     float lat;
//     float pos_N;
//     float lon;
//     float pos_E;
//     float radius;
//     bool turnLeft;
//     float heading_desired;
// };
// extern std::vector<target_s> target_list;
// //target_list.reserve(5);
// extern bool new_targets;
// extern int target_idx;

struct target_s {
    float lat;
    float pos_N;
    float lon;
    float pos_E;
    float radius;
    bool turnLeft;
    float heading_desired;
};
extern std::vector<target_s> target_list;

extern bool new_targets;
extern int target_idx;
extern bool run_path_planner;

std::vector<target_s> order_targets(std::vector<float> tgt_x_list, std::vector<float> tgt_y_list, float in_x, float in_y, \
                               float in_v_x, float in_v_y) ;
void low_loop();
#endif /* AA241X_SLOW_H_ */
