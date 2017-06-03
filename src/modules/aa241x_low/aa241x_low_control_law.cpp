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
 * @file aa241x_low.cpp
 *
 * Secondary control law file for AA241x.  Contains control/navigation
 * logic to be executed with a lower priority, and "slowly"
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */


// include header file
#include "aa241x_low_control_law.h"
#include "aa241x_low_aux.h"

#include <uORB/uORB.h>

using namespace aa241x_low;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 *
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */

std::vector<float> lat_vals(3);
std::vector<float> lon_vals(3);
int target_idx = 0;
bool new_targets = false;
std::vector<target_s> target_list;

//target_list.reserve(5);

std::vector<target_s> order_targets(std::vector<float> tgt_lats, std::vector<float> tgt_lons, float lat_start, float lon_start, \
                               float lat_v, float lon_v) {
    int N = lat_vals.size();
    std::vector<int> idxs(N);
    std::iota(std::begin(idxs), std::end(idxs), 0);
    float min_time = 0;
    float min_dist = 0;
    std::vector<int> best_order(N);
    while (std::next_permutation(std::begin(idxs), std::end(idxs))) {
        float dist = 0;
        float time = 0;
        for (int i; i<N; i++) {


        }

    }


}

bool first_run = true;
void low_loop()
{

	float my_float_variable = 0.0f;		/**< example float variable */
    if (first_run) {
        target_list.reserve(5);
        target_s target1;
        target_s target2;
        target_s target3;

        target1.heading_desired = 0.0;
        target1.turnLeft = true;
        target1.pos_E = 0.0f; //CHANGE THIS!
        target1.pos_N = 0.0f; //CHANGE THIS!
        target_list.push_back(target1);

        target2.heading_desired = 1.5;
        target2.turnLeft = true;
        target2.pos_E = 10.0f; //CHANGE THIS!
        target2.pos_N = 10.0f; //CHANGE THIS!
        target_list.push_back(target2);

        target3.heading_desired = -1.5;
        target3.turnLeft = true;
        target3.pos_E = 10.0f; //CHANGE THIS!
        target3.pos_N = 10.0f; //CHANGE THIS!
        target_list.push_back(target3);

        first_run = false;
        new_targets = true;
    }

	// getting high data value example
	// float my_high_data = high_data.field1;

	// setting low data value example
	low_data.field1 = my_float_variable;


}
