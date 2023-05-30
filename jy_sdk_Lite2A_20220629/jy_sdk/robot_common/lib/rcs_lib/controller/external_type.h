/*
*  Copyright (c) 2018--2019 Deep Robotics. All rights reserved.
*  License(GPL)
*  Developer:
*    2018.01 - Now: Zhang Jiang-Yuan, Email: r03921090@ntu.edu.tw
*/

#ifndef INTERFACE_EXTERNAL_TYPE_H_
#define INTERFACE_EXTERNAL_TYPE_H_

#include <iostream>
#include <cmath>

using namespace std;

const int kMaxScopeNum = 140;
typedef double scope_output[kMaxScopeNum];
const double kPI = 3.1415926535898;
const double kDegree2Radian = kPI / 180;

typedef struct
{
	double value[3];
} OneTypeLegData;

typedef struct
{
	OneTypeLegData fl_pos;
	OneTypeLegData fr_pos;
	OneTypeLegData hl_pos;
	OneTypeLegData hr_pos;
	OneTypeLegData fl_vel;
	OneTypeLegData fr_vel;
	OneTypeLegData hl_vel;
	OneTypeLegData hr_vel;
	OneTypeLegData fl_torque;
	OneTypeLegData fr_torque;
	OneTypeLegData hl_torque;
	OneTypeLegData hr_torque;
} DataLegs;

typedef struct
{
	double roll; double pitch; double yaw;
	double rol_vel; double pit_vel; double yaw_vel;
	double acc_x; double acc_y; double acc_z;
} DataGyro;


#ifndef OUTSIDER
typedef struct ControlCommands
{
	union {bool   button_option;    bool   is_stand;};
	union {bool   button_start;     bool   switch_to_force_model;};
	union {bool   button_A;         bool   turn_over_flag;};
	union {bool   button_X;         bool   raise_head_flag;};
	union {bool   button_Y;         bool   start_action_flag;};
	union {bool   button_LB;        bool   change_swing_leg_flag;};
	union {bool   button_RB;        bool   start_pronk_flag;};
	union {bool   Dpad_up_and_down; bool   is_vision_mode;};       ///< up: true, down: false
	union {int    Dpad_left;        int    gait;};                 ///< switch: 2, 3, 6
	union {int    Dpad_right;       int    jump_type;};      ///< single-click: false, double-click: true
	union {int    LT_and_RT;        int    velocity_control_y;};   ///< hold LT: 1, hold RT: -1, loosen: 0
	double left_axis_x;         ///<                  value: between -1 and 1, resolution: 1/32767
  double left_axis_y;         ///< Forward speed.  value: between -1 and 1, resolution: 1/32767
  double right_axis_x;        ///<                  value: between -1 and 1, resolution: 1/32767
  double right_axis_y;        ///< Turn speed.      value: between -1 and 1, resolution: 1/32767
	bool is_app_lose;
	bool app_start_stop_flag;

	bool is_stop_button_on;
	bool is_gyro_data_lose;
	bool is_drive_heat_warning;
	bool is_wifi_lose;
	bool is_joystick_connect;
	bool is_heat_data_lose;
	bool is_slam_lose;
	bool is_battary_low;
	bool is_zeroposition;
	bool is_keeprun;
	bool is_twist_body_mode;
	bool recover_direction_flag;
	bool start_backflip_flag;
	bool start_sideflip_flag;
	bool is_backflip_mode;
	bool greeting_flag;
	bool low_height_flag;
	int greeting_mode;
	int gait_mode;
	int vel_mode;
	int height_mode;
	int shot_cmd;

	int  is_driver_error;
	int  motor_temperature;
	int  driver_temperature;

	bool   is_gesture_mode;
	int    gesture_value;
	bool   vision_is_reach_charge_area;
	double vision_forward_vel;
	double vision_side_vel;
	double vision_turning_angle;
	int    vision_body_state;
	double vision_delay_time;
	double vision_location_x;
	double vision_location_y;
	double vision_location_theta;
	double vision_goal_x;
	double vision_goal_y;
	double vision_goal_theta;
	bool vision_is_slam_mode;
	bool vision_is_voice_mode;
	bool vision_is_obstacle_distance_mode;
	bool is_app_voice_cmd;
	double lidar_location_x;
	double lidar_location_y;
	double lidar_location_theta;
  bool retry_charge;
  bool charge_over;
	int cmd_quick_roll;
	int voice_value;
	int soft_emergency_cnt;
	int save_data_wait_cnt;
	bool is_save_data;
	int hand_signal;
	double slam_yaw_correct;
	double slam_yaw_vel;
  double slam_x_direction_vel;
  double slam_y_direction_vel;
  double slam_yaw_acc;
  double slam_x_direction_acc;
  double slam_y_direction_acc;
  int friction_state;


  double target_pos_x;
  double target_pos_y;
  double target_theta;

  double current_lidar_pos_x;
  double current_lidar_pos_y;
  double current_lidar_theta;

  double grid_map_posture[4];

  double obstacle_distance[4];
} ControlCommands;

typedef bool ItemValid;
typedef struct
{
	struct { ItemValid valid; double value; }forward_velocity_offset;
  	struct { ItemValid valid; double value; }side_velocity_offset;
  	struct { ItemValid valid; double value; }stand_height;
    double backflip_hip_kp[4];
	double backflip_knee_kp;
    double sideflip_abad_kp;
	double sideflip_hip_kp;
    double sideflip_knee_kp;
}MotionConfig;

namespace task_phase{
  enum TaskPhase{
    kNoTask     = 0,
    kProcessing = 1,
    kFinish     = 2,
  };
};

extern MotionConfig inner_motion_config;

// task_state.task_step[task_state.task_id] read task step value
struct TaskState
{
	union {
		struct {
			int task_id;   // 0: no task,  1+: some task
			int task_step_real[31];    // reserved
		};
		int task_step[32];  // 0:idle, 1:processing, 2:finish
	};
};

struct RobotJointAngle{
	double joint_angle[12];
};

struct RobotJointVel{ 
	double joint_vel[12]; 
}; 

struct RobotActionState{
	int state;
	int value;
};

struct RobotActionUpload{
	int voice_flag;
};

struct RobotCmdUpload{
	double left_axis_y;
	double left_axis_x;
	double right_axis_x;
	double goal_vel_x;
	double goal_vel_y;
	double goal_yaw_angle_vel;
};

struct RobotStateUpload{
	int robot_basic_state;
	int robot_gait_state;
	double rpy[3];
	double rpy_vel[3];
	double xyz_acc[3];
	double pos_world[3];
	double vel_world[3];
	double vel_body[3];
	//double joint_angle[12];
	unsigned touch_down_and_stair_trot;
	bool is_charging;
	unsigned error_state;//[0]low battery; [1]hight temp
	int robot_motion_state;
	double battery_level;
	int task_state;
	bool is_robot_need_move;
	bool zero_position_flag;
};

struct DownLoadParameters{
  double kp[12];
  double kd[12];
  double t_ff[12];
  double goal_angle[12];
  double goal_angle_vel[12];
	double goal_angle_comp[12];
};
struct UpParameters{
  double kp[3];
  double kd[3];
  double t_ff[12];
  double goal_angle[12];
  double goal_angle_vel[12];
};
extern TaskState task_state;
extern RobotActionState robot_action_state;
extern int robot_mile_meter[6];
extern RobotStateUpload robot_state_upload;
extern DownLoadParameters download_parameters;
extern UpParameters up_parameters;
extern RobotJointAngle robot_joint_angle;
extern RobotJointVel robot_joint_vel; 
extern RobotActionUpload robot_action_upload; 
extern RobotCmdUpload robot_cmd_upload; 
#else
struct ControlCommands
{
	bool   button_option;
	bool   button_start;
	bool   button_Y;
	bool   button_X;
	bool   button_A;
	bool   button_LB;
	bool   button_RB;
	bool   Dpad_up_and_down;   ///< up: true, down: false
	int    Dpad_left;          ///< switch: 2, 3, 6
	int    Dpad_right;         ///< single-click: false, double-click: true
	int    LT_and_RT;          ///< hold LT: 1, hold RT: -1, loosen: 0
	double left_axis_x;        ///< value: between -1 and 1
	int    right_axis_y;       ///< value: -1, 0, 1

  bool is_app_lose;
  bool app_start_stop_flag;
	bool is_stop_button_on;
	bool is_gyro_data_lose;
	bool is_drive_heat_warning;
	bool is_wifi_lose;
	bool is_joystick_connect;
	bool is_heat_data_lose;
	int  is_driver_error;
	int  motor_temperature;
};
#endif

#endif  // INTERFACE_EXTERNAL_TYPE_H_
