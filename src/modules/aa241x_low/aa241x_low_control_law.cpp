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
#define PI 3.14159265f
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
bool run_path_planner = false;

//target_list.reserve(5);

std::vector<target_s> order_targets(std::vector<float> tgt_x_list, std::vector<float> tgt_y_list, float in_x, float in_y, \
                               float in_v_x, float in_v_y) {
    int N = tgt_x_list.size();
    std::vector<int> idxs(N);
    std::iota(std::begin(idxs), std::end(idxs), 0);
    float min_time = INFINITY;
//    float min_dist = 0;
    std::vector<int> best_order(N);
    std::vector<float> best_headings(N);
    std::vector<bool> best_turn_left(N);
    float radius = 10;
    float v_turn = 12;
    float v_straight = 15;
//    std::cout << "Before while loop\n";
    do {
        std::vector<float> headings(N);
        std::vector<bool> turn_left(N);
//        float dist = 0;
        float time = 0;
        float start_x = in_x;
        float start_y = in_y;
        float start_v_x = in_v_x;
        float start_v_y = in_v_y;
//        std::cout << "Order: ";
        for (int i = 0; i<N; i++) {
//            std::cout << idxs[i] << " ";
            float tgt_x = tgt_x_list[idxs[i]];
            float tgt_y = tgt_y_list[idxs[i]];
            float v_tgt_x = tgt_x - start_x;
            float v_tgt_y = tgt_y - start_y;
            float norm_dist = (start_v_x * v_tgt_x + start_v_y * v_tgt_y) / (powf(start_v_x, 2.0f) + powf(start_v_y, 2.0f));
            float int_x = norm_dist * start_v_x + start_x;
            float int_y = norm_dist * start_v_y + start_y;
            float v_int_x = tgt_x - int_x;
            float v_int_y = tgt_y - int_y;
            float mask_x;
            float mask_y;
            if ((int)std::ceil(std::abs(start_v_y)) != 0 && (int)std::ceil(std::abs(v_int_x)) != 0) {
                mask_x = (v_int_x / start_v_y < 0) ? -1.0f : 1.0f;
            } else {
                mask_x = (v_int_x < 0) ? 1.0f : -1.0f;
            }
            if ((int)std::ceil(std::abs(start_v_x)) != 0 && (int)std::ceil(std::abs(v_int_y)) != 0) {
                mask_y = (v_int_y / start_v_x < 0) ? -1.0f : 1.0f;
            } else {
                mask_y = (v_int_y < 0) ? 1.0f : -1.0f;
            }
            float vxc = start_v_y * mask_x;
            float vyc = start_v_x * mask_y;
            float xc = start_x + radius * vxc / sqrtf(powf(vxc, 2.0f) + powf(vyc, 2.0f));
            float yc = start_y + radius * vyc / sqrtf(powf(vxc, 2.0f) + powf(vyc, 2.0f));
            float xd = tgt_x - xc;
            float yd = tgt_y - yc;
            //roots to find k
            float a_k = 8.0f * xc * tgt_x + 4.0f * powf(yd, 2.0f) - \
                4.0f * (powf(xc,2.0f) + powf(yc, 2.0f) + powf(tgt_y, 2.0f) - 2.0f * yc * tgt_y - powf(radius, 2.0f)) - \
                4.0f * powf(tgt_x, 2.0f);
            float b_k = 8.0f * yd * xd;
            float c_k = 4.0f * powf(xc, 2.0f) - 4.0f * (powf(xc,2.0f) + powf(yc, 2.0f) + powf(tgt_y, 2.0f) - 2.0f * yc * tgt_y - powf(radius, 2.0f));

            float k1 = (-b_k + sqrtf(powf(b_k, 2.0f) - 4.0f * a_k * c_k + 1e-7f)) / (2.0f * a_k);
            float k2 = (-b_k - sqrtf(powf(b_k, 2.0f) - 4.0f * a_k * c_k + 1e-7f)) / (2.0f * a_k);

            float a1 = powf(k1, 2.0f) + 1.0f;
            float a2 = powf(k2, 2.0f) + 1.0f;
            float b1 = -2.0f * tgt_x * powf(k1, 2.0f) + 2.0f * yd * k1 - 2.0f * xc;
            float b2 = -2.0f * tgt_x * powf(k2, 2.0f) + 2.0f * yd * k2 - 2.0f * xc;

            float x1 = -b1 / (2.0f * a1);
            float x2 = -b2 / (2.0f * a2);
            float y1 = k1 * (x1 - tgt_x) + tgt_y;
            float y2 = k2 * (x2 - tgt_x) + tgt_y;

            float v1x = tgt_x - x1;
            float v2x = tgt_x - x2;
            float v1y = tgt_y - y1;
            float v2y = tgt_y - y2;

            float ang1raw = atan2f (y1 - yc,x1 - xc) * 180.0f / PI;
            float ang2raw = atan2f (y2 - yc,x2 - xc) * 180.0f / PI;
            float angstart = atan2f (start_y - yc,start_x - xc) * 180.0f / PI;
            float ang1 = (angstart*mask_x + ang1raw*mask_y);
            float ang2 = (angstart*mask_x + ang2raw*mask_y);
            ang1 = (ang1 < 0.0f) ? ang1 + 360.0f : ang1;
            ang2 = (ang2 < 0.0f) ? ang2 + 360.0f : ang2;

            float angmin = (ang1 < ang2) ? ang1 : ang2;
            float xmin = (ang1 < ang2) ? x1 : x2;
            float ymin = (ang1 < ang2) ? y1 : y2;
            float vminx = (ang1 < ang2) ? v1x : v2x;
            float vminy = (ang1 < ang2) ? v1y : v2y;

            float d_turn = angmin * PI / 180.0f * radius;
            float d_straight = sqrtf(powf(tgt_x - xmin, 2.0f) + powf(tgt_y - ymin, 2.0f));
            float t_turn = d_turn / v_turn;
            float t_straight = d_straight / v_straight;
            headings[i] = atan2f(vminx, vminy);
            turn_left[i] = ((mask_x <  0.0f));
//            dist += d_turn;
//            dist += d_straight;
            time += t_turn;
            time += t_straight;
            start_x = tgt_x;
            start_y = tgt_y;
            start_v_x = vminx / sqrtf(powf(vminx, 2.0f) + powf(vminy, 2.0f));
            start_v_y = vminy / sqrtf(powf(vminx, 2.0f) + powf(vminy, 2.0f));

        }
        if (time < min_time) {
            min_time = time;
//            min_dist = dist;
            best_order = idxs;
            best_headings = headings;
            best_turn_left = turn_left;
        }
//        std::cout << "\n";

    } while (std::next_permutation(std::begin(idxs), std::end(idxs))) ;
    std::vector<target_s> targets_to_output;
    targets_to_output.reserve(N);
    for (int i = 0; i<N; i++) {
        target_s temp_target;
        temp_target.yaw = best_headings[i];
        temp_target.turnLeft = best_turn_left[i];
        temp_target.pos_E = tgt_x_list[best_order[i]];
        temp_target.pos_N = tgt_y_list[best_order[i]];
        temp_target.radius = 2.5f;
        //temp_target.radius = plume_radius[best_order[i]];
        targets_to_output.push_back(temp_target);
//        std::cout << "Heading " << i << ": " << best_headings[i] << "\n";
//        std::cout << "Turn Left " << i << ": " << best_turn_left[i] << "\n";
//        std::cout << "East Pos " << i << ": " << tgt_x_list[best_order[i]] << "\n";
//        std::cout << "North Pos " << i << ": " << tgt_y_list[best_order[i]] << "\n";
    }
//    std::cout << "Best Order: ";
//    for (int i = 0; i<N; i++) {
//        std::cout << best_order[i] << " ";
//    }
//    std::cout << "\nBest Time: " << min_time << "\nBest Distance " << min_dist << "\n" ;
    return targets_to_output;

}


int32_t prev_phase = phase_num - 1;
bool first_run = true;
void low_loop()
{

//	float my_float_variable = 0.0f;		/**< example float variable */
 /*   if (first_run) {
        target_list.reserve(5);
        target_s target1;
        target_s target2;
        target_s target3;

        target1.yaw = 0.0;
        target1.turnLeft = true;
        target1.pos_E = 0.0f; //CHANGE THIS!
        target1.pos_N = 0.0f; //CHANGE THIS!
        target_list.push_back(target1);

        target2.yaw = 1.5;
        target2.turnLeft = true;
        target2.pos_E = 10.0f; //CHANGE THIS!
        target2.pos_N = 10.0f; //CHANGE THIS!
        target_list.push_back(target2);

        target3.yaw = -1.5;
        target3.turnLeft = true;
        target3.pos_E = 10.0f; //CHANGE THIS!
        target3.pos_N = 10.0f; //CHANGE THIS!
        target_list.push_back(target3);

        first_run = false;
        new_targets = true;
        run_path_planner = false;
    }
    if (run_path_planner) {
        std::vector<target_s> targets;
        std::vector<float> tgt_x_list {-100.0f, 100.0f, 0.0f, 75.0f, -80.0f};
        std::vector<float> tgt_y_list {100.0f, 100.0f, 200.0f, 150.0f, 50.0f};
        float in_x = 0.0f;
        float in_y = 0.0f;
        float in_v_x = 1.0f;
        float in_v_y = 0.0f;
    //    std::cout << "Calling Targets \n";
        targets = order_targets(tgt_x_list, tgt_y_list, in_x, in_y, \
                                   in_v_x, in_v_y);
        new_targets = true;
        run_path_planner = false;. 
    }*/
/*
    if (in_mission && prev_phase != phase_num) { //New targets available
        int j = 0;
        std::vector<float> tgt_y_list;
        std::vector<float> tgt_x_list;
        while((int)std::ceil(std::abs(plume_N[j])) != 0) {
            tgt_x_list.push_back(plume_E[j]);
            tgt_y_list.push_back(plume_N[j]);
            j++;
        }
        float in_x = position_E;
        float in_y = position_N;
        float in_v_x = std::sin(ground_course);
        float in_v_y = std::cos(ground_course);
        target_list = order_targets(tgt_x_list, tgt_y_list, in_x, in_y, \
                                   in_v_x, in_v_y);
        new_targets = true;
        prev_phase = phase_num;
    }
  */  
    /*
    if (in_mission && prev_phase != phase_num) { //New targets available
        int j = 0;
        std::vector<float> tgt_y_list;
        std::vector<float> tgt_x_list;
        while((int)std::ceil(std::abs(plume_N[j])) != 0) {
            target_s target1;
            target1.yaw = 0.0;
            target1.turnLeft = true;
            target1.pos_E = plume_E[j]; //CHANGE THIS!
            target1.pos_N = plume_N[j]; //CHANGE THIS!
            target_list.push_back(target1);
            j++;
        }
        
    }
    */
    
    float test_N[4] = {position_N, position_N+7.0f, position_N-7.0f, 0.0f};
    float test_E[4] = {position_E+7.0f, position_E, position_E-7.0f, 0.0f};
    if (first_run) { //New targets available
        int j = 0;
        std::vector<float> tgt_y_list;
        std::vector<float> tgt_x_list;
        while((int)std::ceil(std::abs(test_N[j])) != 0) {
            tgt_x_list.push_back(test_E[j]);
            tgt_y_list.push_back(test_N[j]);
            j++;
        }
        float in_x = position_E;
        float in_y = position_N;
        float in_v_x = std::sin(ground_course);
        float in_v_y = std::cos(ground_course);
        target_list = order_targets(tgt_x_list, tgt_y_list, in_x, in_y, \
                                   in_v_x, in_v_y);
        new_targets = true;
        first_run = false;
    }

	// getting high data value example
	// float my_high_data = high_data.field1;

	// setting low data value example
	//low_data.field1 = my_float_variable;


}

// int main() {
///*
//    for (int i; i<targets.size(); i++) {
//
//        std::cout << "Heading " << i << ": " << targets[i].heading_desired << "\n";
//        std::cout << "Turn Left " << i << ": " << targets[i].turnLeft << "\n";
//        std::cout << "East Pos " << i << ": " << targets[i].pos_E << "\n";
//        std::cout << "North Pos " << i << ": " << targets[i].pos_N << "\n";
//    }
//    std::cout.flush();
//    */
//    return targets.size();
//}
