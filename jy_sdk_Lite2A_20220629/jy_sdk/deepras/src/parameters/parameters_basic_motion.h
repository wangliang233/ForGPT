
#ifndef PARAMETERS_BASIC_MOTION_H_
#define PARAMETERS_BASIC_MOTION_H_

namespace basic_motion {
	enum RobotStateMap {
		kWaitForStanding = 1,
		kFoldHip=2,
		kFoldKnee=3,
		kPrepareMotion = 4,

		kStandUpMotion = 5,
		kVirtualForceStanding = 6,
		kStandDownMotion = 7,
		kLoseControlProcess = 8,
		kAbnormalPostureRecovery = 9,
		kRaiseHead = 10,
		kTurnOver = 11,
		kGestureInteractionMotion = 12,
		kAutoChargeLocation = 13,
		kVoiceInteractionMotion = 14,
    kTestPre = 15,
    kTest = 16,
    kZeroPosition=17,
	kBackflip = 18,
	};
	const double kInitialPoseTime = 0.4;
	const double kStandUpTime = 1.2;

	const double kRaiseHeadAngle = -20 * kDegree2Radian;
	const double kGoalBodyHeight = 0.45;
	const double kGoalBodyHeightJYM = 0.36;
  const double kGoalBodyHeightJYS = 0.25;
	const double kGoalBodyHeightJYC = 0.45;


	typedef struct
	{
		bool is_gesture_stand_up;
		bool is_first_start_finish;
		bool is_switch_state_record;
		bool stand_up_down_flag_record;
		bool turn_over_flag_record;
		bool backflip_flag_record;
		bool prepare_backflip_flag_record;
		bool is_backflip_finish;
		bool is_backflip_to_stand;
		bool app_start_stop_flag_record;
		bool is_app_start_action;
		bool raise_head_flag_record;
		bool start_pronk_flag_record;
		bool start_stop_action_flag_record;
		bool switch_to_force_mode_flag_record;
		bool is_stand_up_finished;
		bool is_switch_to_force_finished;
		bool is_switch_to_raise_head_finished;
		bool zeroposition_flag_record;
		bool app_zeroposition_flag_record;
		bool is_switch_to_zeroposition_finished;
		bool is_after_first_start;
		int state;
		int state_posture_recovery;
		int state_turn_over;
		int state_backflip;
		double state_initial_time;
		double switch_to_force_control_time;
		int gesture_value;
		int turn_over_flag;
		int standupfold_flag;
		double fold_time;
		double turn_over_time;

		double target_auto_charge_theta;
		double target_auto_charge_x;
		double target_auto_charge_y;
		int auto_charge_phase;

		double goal_yaw_angle;

		Vec3 fl_xyz;
		Vec3 fl_xyz_vel;
		Vec3 fr_xyz;
		Vec3 fr_xyz_vel;
		Vec3 hl_xyz;
		Vec3 hl_xyz_vel;
		Vec3 hr_xyz;
		Vec3 hr_xyz_vel;

		Vec3 fl_initial_angle;
		Vec3 fr_initial_angle;
		Vec3 hl_initial_angle;
		Vec3 hr_initial_angle;

		Vec3 cent_angle;
		Vec3 cent_xyz;
		double raise_head_angle;
		bool quick_roll;
		bool pos_recovery_to_stand_up_flag;
		bool stand_down_to_turn_over_flag;
		bool turn_over_to_pos_recovery_flag;
		bool lose_control_to_pos_recovery_flag;
		bool stand_up_to_force_control_flag;
		bool in_quick_roll;
		bool quick_roll_finish;
		int gesture_count;
		double gesture_yaw;
		double gesture_height;
		double gesture_pitch;

		bool is_voice_stand_up;
		int voice_motion_count;
		double voice_yaw;
		double voice_height;
		double voice_pitch;
		int last_voice_value;

		int jump_type;

		double half_control_period_flag;
		double start_to_flip;
		int h_hit_gnd_flag;
		int f_hit_gnd_flag;
		Eigen::Matrix4Xd jointHistory;
		Eigen::VectorXd pTarget, dTarget,tauFF;

	}DataGlobal;
}
#endif  //PARAMETERS_BASIC_MOTION_H_
