

#ifndef PARAMETERS_INNER_TYPE_H_
#define PARAMETERS_INNER_TYPE_H_

#include "Eigen/Dense"
#include "controller/external_type.h"

#include "iostream"
#include <cmath>
#include <iomanip>

using namespace Eigen;
using namespace std;

typedef Matrix< double, 3, 1> Vec3;
typedef Matrix< double, 4, 1> Vec4;
typedef Matrix< double, 6, 1> Vec6;
typedef Matrix< double, 6, 6> Vec66;
typedef Matrix< double, 3, 3> Vec33;

typedef struct
{
	double hipx_kp;
	double hipx_kd;
	double hipy_kp;
	double hipy_kd;
	double knee_kp;
	double knee_kd;
	double tor_lim[3];
	double gravity_coef1;
	double gravity_coef2;
	double gravity_coef3;
	double low_vel;
	//GroundContactDetection
	double d1;
	double d2;
	double m1;
	double m2;
	double I0;
	double I0_comp;
	double I1;
	double I1_comp;
	double I2;
	double I2_comp;
	//GroundContactDetection
} OneLegControlParameter;

typedef struct
{
	double fx_kp;
	double fx_kd;
	double fy_kp;
	double fy_kd;
	double fz_kp;
	double fz_kd;
	double roll_kp;
	double roll_kd;
	double pitch_kp;
	double pitch_kd;
	double yaw_kp;
	double yaw_kd;
	bool is_support_posture_recovery;
} BodyControlParameter;

typedef struct
{
	OneLegControlParameter leg;
	BodyControlParameter body;
	BodyControlParameter stand;
} ControlParameter;

//class ControlParameter{
//public:
//	ControlParameter(OneLegControlParameter _leg,BodyControlParameter _body,BodyControlParameter _stand){
//		leg = _leg;
//		stand = _stand;
//		body = _body;
//	}
//	~ControlParameter(){}
//	OneLegControlParameter leg;
//	BodyControlParameter body;
//	BodyControlParameter stand;
//};

typedef struct {
	double link_length[3];
} OneLegAttribute;

typedef struct
{
	double body_length_x;
	double body_length_y;
	double body_length_z;
	double min_body_height;
	double mass;
}BodyAttribute;

typedef struct
{
	OneLegAttribute fl;
	OneLegAttribute fr;
	OneLegAttribute hl;
	OneLegAttribute hr;
	BodyAttribute body;
}HardwareParameter;

typedef struct
{
	double fl_hipx_integral;
	double fr_hipx_integral;
	double hl_hipx_integral;
	double hr_hipx_integral;
	Vec3 last_v;
	Vec3 current_velocity;
	Vec3 ang_acc;
	Vec3 estimated_force_with_acc;
	string limit_var[10];
	double limit_var_value[10];
}DataGlobalOneLeg;

typedef struct
{
	bool yaw_angle_first_record;
	double last_cycle_yaw_angle;
	int yaw_angle_cycle;
	bool is_gyro_connect;
	int last_control_state_record;
	int last_target_robot_state_record;
	bool start_stop_action_flag_record;
	bool raise_head_flag_record;
	bool start_pronk_flag_record;
	bool app_start_stop_falg_record;
	double robot_woking_state_judge_output_time_record;
	bool is_robot_continue_to_charge;
	double is_robot_leave_charge_time_record;
	double robot_continue_to_charge_abs_time;
	double vision_location_output;
	double vision_location_x_record;
	double vision_location_y_record;
	double vision_location_connect_time_record;
	double gesture_move_forward_time_record;
	double gesture_move_backward_time_record;
	double gesture_turn_right_time_record;
	double gesture_turn_left_time_record;
	bool is_gesture_movement;
	bool is_voice_movement;
	bool is_gesture_forward_move_stop;
	bool is_gesture_back_move_stop;
	int voice_move_value;
	int last_voice_value;
	double last_voice_value_time_record;
	double last_standup_time_record;
	bool isSlamNavigationStart;
	string scope_object[kMaxScopeNum][2];
	int soft_emergency_cnt;
	double last_back_obstacle;
}DataGlobalPreProcess;

typedef struct
{
	double forward_vel;
	double side_vel;
	double body_height;
	double body_vel;
	double roll_posture;
	double roll_vel;
	double pitch_posture;
	double pitch_vel;
	double yaw_posture;
	double yaw_vel;
	bool is_robot_lose_control;
}RobotState;

typedef struct
{
	double low;
	double high;
}Range;

typedef struct
{
	Range forward_vel;
	Range side_vel;
	Range body_height;
	Range roll;
	Range pitch;
}RobotSwithState;

typedef struct
{
	bool is_robot_can_continue_work;
	bool stand_up_down_flag;
	bool app_start_stop_flag;
	bool is_app_connect;
	bool raise_head_flag;
	bool switch_to_force_mode_flag;
	bool start_stop_action_flag;
	bool turn_over_falg;
	bool zeroposition_flag;
	bool is_app_zeroposition;
	bool recover_direction_flag;
	bool is_robot_continue_to_charge;
	bool is_gesture_mode;
	bool start_pronk_flag;
	bool start_backflip_flag;
	bool is_backflip_mode;
	int target_robot_state;
	int gesture_value;  // 0:No detect, 1:Stand up, 2:Stand down, 3:Move forward, 4:Move backward, 5:Stop
	double vision_location_x;
	double vision_location_y;
	double vision_location_theta;
	bool retry_charge;
	bool charge_over;
	bool cmd_quick_roll;
	int voice_value;
	int shot_cmd;
	double yaw;
	double pitch;
	double z;
	double roll;
	bool is_soft_emergency_pressed;
	bool is_robot_can_move;
}CommandsBasicMotion;

typedef struct
{
	bool is_robot_switch_state;
	double user_forward_vel;   // [-0.6, 1.2]
	double user_side_vel;	   // [-0.5, 0.5]
	int user_turning_control;  // {-1, 0, 1}
	bool change_swing_leg_flag;
	bool is_pronk_single;
	bool change_body_height_flag;
	bool adjust_peroid_flag;
	bool is_vision_mode;
	double vision_forward_vel;
	double vision_side_vel;
	double vision_turning_vel;
	double vision_turning_angle;
	double vision_delay_time;
	bool is_landmark_mode;
	double vision_location_x;
	double vision_location_y;
	double vision_location_theta;
	bool vision_is_slam_mode;
	bool vision_is_voice_mode;
	bool vision_is_obstacle_distance_mode;
	double slam_yaw_vel;
  double slam_x_direction_vel;
  double slam_y_direction_vel;
  double slam_yaw_acc;
  double slam_x_direction_acc;
  double slam_y_direction_acc;
  int    friction_state;
double last_back_obstacle;

	double trot_standing_time;
	double trot_fly_phase_time;
	int jump_type;
	int pronk_type;
	int walk_type;

	int gait_mode;

	double obstacle_distance[4];
}CommandsMotion;

typedef struct
{
	int target_robot_state;
	CommandsBasicMotion basic;
	CommandsMotion motion;
}InnerControlCommands;
#endif  // PARAMETERS_INNER_TYPE_H_
