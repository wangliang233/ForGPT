/*
*  Copyright (c) 2018--2019 Deep Robotics. All rights reserved.
*  License(GPL)
*  Developer:
*    2018.01 - Now: Zhang Jiang-Yuan, Email: r03921090@ntu.edu.tw
*/

#ifndef INTERFACE_MAIN_H_
#define INTERFACE_MAIN_H_

#include "external_type.h"
#include <inttypes.h>

#include "Eigen/Core"
#ifndef OUTSIDER
void MotionInitial(const string &robot_id, const string &robot_type, const MotionConfig &motion_config);
extern Eigen::MatrixXf mat0,mat1;
string ShowVersion();


void ScopeNamesOutput();

#else
void MotionInitial(const string &robot_id);
#endif

void Motion(const DataLegs *leg_input, const DataGyro &gyro_data, const double &time_stamp, const double &cycle_time, const ControlCommands &control_commands,
			scope_output *scope_output, DataLegs *leg_output);

void Plan();

/// return a millisecond time value, min = 10ms, max = 2s,invalid = 0
uint32_t PlanPeroid();

#endif  // INTERFACE_MAIN_H_
