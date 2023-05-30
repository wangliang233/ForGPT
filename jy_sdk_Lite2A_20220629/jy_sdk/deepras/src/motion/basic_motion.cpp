
#include "basic_motion.h"
#include <fstream>
#include "Eigen/Dense"
using namespace basic_motion;

DataGlobal BasicMotion::global_;

BasicMotion::BasicMotion(LegControl *fl,
						 LegControl *fr,
						 LegControl *hl,
						 LegControl *hr,
						 RobotState *goal_robot_state,
						 RobotState *actual_robot_state):
	fl_(fl),
	fr_(fr),
	hl_(hl),
	hr_(hr),
	goal_robot_state_(goal_robot_state),
	actual_robot_state_(actual_robot_state)
{
	for(int i=0;i<3;++i){
		filter_input[i] = 0;
		filter_output[i] = 0;
	}
}

BasicMotion::~BasicMotion()
{

}

void BasicMotion::ConstParameterInitial(const string &robot_id, const string &robot_type)
{
	robot_id_ = robot_id.substr(0, 4);
	if (robot_id_ == "JY-S") {
		default_control_parameters_ = kDefaultControlParameterJYS;
		body_attribute_ = kHardwareParameterJYS.body;
	} else {
		while (true) {
			cout << "Const parameters initial failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		}
	}
	if ((robot_id_ == "JY-C" && abs(inner_motion_config.stand_height.value - 0.45) < 0.001)
			|| (robot_id_ == "JY-P" && abs(inner_motion_config.stand_height.value - 0.50) < 0.001)
			|| (robot_id_ == "JY-M"  && abs(inner_motion_config.stand_height.value - 0.36) < 0.001)
      || (robot_id_ == "JY-S"  && abs(inner_motion_config.stand_height.value - 0.30) < 0.001)) {
		is_support_pronk_ = true;
	} else {
		is_support_pronk_ = false;
	}
	if (robot_id_ == "JY-M" || robot_id_ == "JY-L" || robot_id_ == "JY-S" || robot_id_ == "JY-P") {
		is_support_dance_ = true;
	} else {
		is_support_dance_ = false;
	}
	if (robot_id_ == "JY-L" && abs(inner_motion_config.stand_height.value - 0.45) < 0.001) {
		is_support_jump_ = true;
	} else {
		is_support_jump_ = false;
	}
	if (robot_id_ == "JY-L") {
		virtual_force_PD_rate_ = 0.35;
	} else if (robot_id_ == "JY-M") {
		virtual_force_PD_rate_ = 0.15;
	} else if (robot_id_ == "JY-S") {
		virtual_force_PD_rate_ = 0.13;
	} else {
		virtual_force_PD_rate_ = 1.0;
	}
	virtual_force_yaw_kp_ = 5000;
	if (robot_id.substr(0, 5) == "JY-L2") {
		virtual_force_yaw_kp_ = 3500;
	}
	if (robot_id_ == "JY-M") {
		raise_head_stand_height_ = 0.35;
	} else if (robot_id_ == "JY-S") {
    raise_head_stand_height_ = 0.25;
  }else {
		raise_head_stand_height_ = 0.4;
	}
}

void BasicMotion::DataInputProcess(const CommandsBasicMotion &control_commands, const double &abs_run_time, const double &cycle_time, const DataGyro &gyro_data_process)
{
	control_commands_ = control_commands;
	gyro_data_ = gyro_data_process;
	abs_run_time_ = abs_run_time;
	cycle_time_ = cycle_time;
	ActualLegPosCalculation();
	is_robot_switch_state_ = false;
	is_robot_switch_to_dance_ = false;
	is_robot_switch_to_pronk_ = false;
	is_robot_switch_to_jump_ = false;
}
static Vec3 temp_angle;
static bool flag_in = true;
void BasicMotion::MotionProcess()
{
	InitialStateRecord();
//	static bool is_zeroposition_finished = false;
	switch (global_.state) {
		case kWaitForStanding: {
			fl_->TorqueOutputDirectly(0,0,0);
			fr_->TorqueOutputDirectly(0,0,0);
			hl_->TorqueOutputDirectly(0,0,0);
			hr_->TorqueOutputDirectly(0,0,0);
			if ((global_.stand_up_down_flag_record != control_commands_.stand_up_down_flag || global_.app_start_stop_flag_record != control_commands_.app_start_stop_flag
					|| control_commands_.gesture_value == kGestureStandUp || control_commands_.voice_value == kVoiceStandUp
					|| control_commands_.charge_over || control_commands_.retry_charge ) && control_commands_.is_robot_can_continue_work){

				global_.state_initial_time = abs_run_time_;
				FourLegsAngleRecord();
				cout << "Standing up, please wait..." << endl;
				global_.standupfold_flag=0;
				global_.pos_recovery_to_stand_up_flag = 0;
				global_.stand_up_to_force_control_flag = 1;
				global_.state =kPrepareMotion;
				if(!global_.is_after_first_start){
					global_.state_initial_time = abs_run_time_;
					global_.state =kZeroPosition;
					cout << "Switching to zeroposition !!!!!" << endl;
				}
				if (abs(gyro_data_.roll) >= 65 * kDegree2Radian) {
					actual_robot_state_->is_robot_lose_control = true;
					FourLegsGetNowPosition();
					global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
					cout << "[Warning] Robot is lose control, Switch to position control mode, wait a moment to press [STAND UP BUTTON] to let robot lose power!!!" << endl;
					global_.state = kLoseControlProcess;
				}
			}

			if (robot_id_ == "JY-S" && (global_.app_zeroposition_flag_record != control_commands_.is_app_zeroposition
				|| (global_.zeroposition_flag_record != control_commands_.switch_to_force_mode_flag && global_.zeroposition_flag_record != control_commands_.zeroposition_flag))) {
				global_.state_initial_time = abs_run_time_;
				global_.state =kZeroPosition;
				cout << "Switching to zeroposition !!!!!" << endl;
			}

			if ((global_.turn_over_flag_record != control_commands_.turn_over_falg ) && control_commands_.is_robot_can_continue_work && default_control_parameters_.body.is_support_posture_recovery) {
				global_.state_initial_time = abs_run_time_;
				FourLegsAngleRecord();
				global_.state_turn_over = 0;
				cout << "Turning over, please wait..." << endl;
				global_.state = kTurnOver;
				global_.stand_down_to_turn_over_flag = 0;
				global_.turn_over_to_pos_recovery_flag = 1;
			}

			break;
		}

		case kZeroPosition:{
			if(abs_run_time_ - global_.state_initial_time <= 8.5){
				if(abs_run_time_ - global_.state_initial_time <= 2.00){
					double t=abs_run_time_ - global_.state_initial_time;
					double hip_torque_X=1.5, hip_torque_Y=-1.5, knee_torque_X=1.5;
					hip_torque_X *= t/2.00;	hip_torque_Y *= t/2.00;	knee_torque_X *= t/2.00;

					fl_->TorqueOutputDirectly(-hip_torque_X,hip_torque_Y,knee_torque_X);
					fr_->TorqueOutputDirectly(hip_torque_X,hip_torque_Y,knee_torque_X);
					hl_->TorqueOutputDirectly(-hip_torque_X,hip_torque_Y,knee_torque_X);
					hr_->TorqueOutputDirectly(hip_torque_X,hip_torque_Y,knee_torque_X);

				} else {
					fl_->TorqueOutputDirectly(-3,-3,3.5);
					fr_->TorqueOutputDirectly(3,-3,3.5);
					hl_->TorqueOutputDirectly(-3,-3,3.5);
					hr_->TorqueOutputDirectly(3,-3,3.5);
				}
				if (abs_run_time_ - global_.state_initial_time > 8.4) robot_state_upload.zero_position_flag = true;
			}   
			if(abs_run_time_ - global_.state_initial_time > 8.5){
				fl_->TorqueOutputDirectly(0,0,0);
				fr_->TorqueOutputDirectly(0,0,0);
				hl_->TorqueOutputDirectly(0,0,0);
				hr_->TorqueOutputDirectly(0,0,0);
				global_.state_initial_time = abs_run_time_;
				FourLegsAngleRecord();
				global_.app_zeroposition_flag_record = control_commands_.is_app_zeroposition;
				global_.zeroposition_flag_record =  control_commands_.zeroposition_flag;
				//is_zeroposition_finished = true;
				// global_.is_switch_to_zeroposition_finished = 1;
				robot_state_upload.zero_position_flag = false;
				global_.is_after_first_start = true;
				global_.turn_over_flag_record = control_commands_.turn_over_falg ;
				global_.state = kWaitForStanding;
			}
			break;
		}

		case kPrepareMotion: {
			Vec3 goal_pos_right;
			goal_pos_right << 0, -fr_->link_.link_length[0], -body_attribute_.min_body_height;
			if (abs_run_time_ <= global_.state_initial_time + 1) {
				FourLegsSwingToPositionAnglePlan(goal_pos_right, 1);
			} else {
				FourLegsKeepPosition(goal_pos_right);
				FourLegsGetNowPosition();
				global_.state_initial_time = abs_run_time_;
				global_.is_stand_up_finished = false;
				global_.state = kStandUpMotion;//kStandUpMotion;
			}
			break;
		}

		case kStandUpMotion: {
			Vec3 goal_pos_right;
			double standup_time = 1;
			double stand_keep_time = 1;
			goal_pos_right << 0, -fr_->link_.link_length[0], -inner_motion_config.stand_height.value;
			if (global_.is_stand_up_finished == false) {
				FourLegsSwingToPositionPathPlan(goal_pos_right, standup_time);
				if (abs_run_time_ > global_.state_initial_time + stand_keep_time) {
					global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
					global_.switch_to_force_mode_flag_record = control_commands_.switch_to_force_mode_flag;
					global_.is_stand_up_finished = true;
					cout << "Stand up action has finished, waitting to switch to force control mode!" << endl;
          			global_.state_initial_time = abs_run_time_;
				}
			}

			if (global_.is_stand_up_finished == true) {

				FourLegsKeepPosition(goal_pos_right);
				global_.state_initial_time = abs_run_time_;
				SwitchToFourLegsStandRecord();
				global_.switch_to_force_control_time = 0.8;
				global_.is_switch_to_force_finished = false;
				cout << "Switching to force control mode, please wait..." << endl;
				global_.state = kVirtualForceStanding;
				global_.stand_up_to_force_control_flag = 0;
			}
			break;
		}

		case kVirtualForceStanding: {
			double roll_estimation = 0,pitch_estimation = 0;
			double pitch_command = control_commands_.z;
			double yaw_command = -control_commands_.yaw;
			double roll_command = control_commands_.roll;
			double z_command;
			if (global_.is_switch_to_force_finished
					&& abs(roll_estimation) < 5.0 * kDegree2Radian
					&& abs(pitch_estimation) < 5.0 * kDegree2Radian){
					if (control_commands_.z < 0){
						z_command = 0.30 + 0.2*inner_motion_config.stand_height.value*control_commands_.pitch;
					}else{
						z_command = 0.30 + 0.2*inner_motion_config.stand_height.value*control_commands_.pitch;
					}
					CommandFollow(0, 0, z_command, roll_estimation + roll_command * 0.55,pitch_estimation + pitch_command * 0.25, yaw_command* 0.55);
			}

			if (global_.is_switch_to_force_finished == false) {
				if (abs_run_time_ > global_.state_initial_time + global_.switch_to_force_control_time) {
					global_.is_switch_to_force_finished = true;
					global_.start_stop_action_flag_record = control_commands_.start_stop_action_flag;
					cout << "Switch to force control has finished, waitting to start or stand down!" << endl;
				}
			}

			if (abs(gyro_data_.roll) > 50 * kDegree2Radian || abs(gyro_data_.pitch) > 50 * kDegree2Radian) {  //switch to position control model.
				actual_robot_state_->is_robot_lose_control = true;
				global_.state_initial_time = abs_run_time_;
				FourLegsGetNowPosition();
				global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
				cout << "[Warning] Robot is lose control, Switch to position control mode, wait a moment to press [STAND UP BUTTON] to let robot lose power!!!" << endl;
				global_.state = kLoseControlProcess;
			}

			if (global_.stand_up_down_flag_record != control_commands_.stand_up_down_flag)
				{
					global_.state_initial_time = abs_run_time_;
					FourLegsGetNowPosition();
					cout << "Standing down, please wait..." << endl;
					global_.state = kStandDownMotion;
				}
			break;
		}

		case kStandDownMotion: {
			Vec3 goal_pos_right;
			goal_pos_right << 0, -fr_->link_.link_length[0], -body_attribute_.min_body_height;
			if (abs_run_time_ <= global_.state_initial_time + kStandUpTime) {
				FourLegsSwingToPositionPathPlan(goal_pos_right, kStandUpTime);
			} else {
				cout << "Stand down action has finished, waitting to stand up again!!!" << endl;
				global_.state = kWaitForStanding;
				global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
				global_.turn_over_flag_record = control_commands_.turn_over_falg;
				global_.backflip_flag_record = control_commands_.start_backflip_flag;
				global_.prepare_backflip_flag_record = control_commands_.is_backflip_mode;
				global_.app_start_stop_flag_record = control_commands_.app_start_stop_flag;
			}
			break;
		}

		case kLoseControlProcess: {
			robot_action_state.state = performance::kLoseControl;
			double lose_control_kd_rate = 1.5;
			fl_->SwingToPosition(global_.fl_xyz, global_.fl_xyz, global_.fl_xyz, 0, "FL", 0 , lose_control_kd_rate);
			fr_->SwingToPosition(global_.fr_xyz, global_.fr_xyz, global_.fr_xyz, 0, "FR", 0 , lose_control_kd_rate);
			hl_->SwingToPosition(global_.hl_xyz, global_.hl_xyz, global_.hl_xyz, 0, "HL", 0 , lose_control_kd_rate);
			hr_->SwingToPosition(global_.hr_xyz, global_.hr_xyz, global_.hr_xyz, 0, "HR", 0 , lose_control_kd_rate);
			if (abs_run_time_ > (global_.state_initial_time + 2.0)){
				fl_->TorqueOutputDirectly(0,0,0);
				fr_->TorqueOutputDirectly(0,0,0);
				hl_->TorqueOutputDirectly(0,0,0);
				hr_->TorqueOutputDirectly(0,0,0);
			} 			
			if ( abs_run_time_ > (global_.state_initial_time + 2.5) && default_control_parameters_.body.is_support_posture_recovery && control_commands_.is_robot_can_continue_work) {
				cout << "Robot is lose power, wait a moment to press [STAND UP BUTTON] to recover the posture!!!" << endl;
				global_.state_initial_time = abs_run_time_;
				global_.state_posture_recovery = 0;
				global_.state = kAbnormalPostureRecovery;
			}
			break;
		}

		case kAbnormalPostureRecovery: {
			AbnormalPostureRecovery();
			if (!actual_robot_state_->is_robot_lose_control) {
				global_.state_initial_time = abs_run_time_;
				global_.state = kWaitForStanding;
				global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
				global_.turn_over_flag_record = control_commands_.turn_over_falg;
				global_.backflip_flag_record = control_commands_.start_backflip_flag;
				global_.prepare_backflip_flag_record = control_commands_.is_backflip_mode;
			}
			break;
		}

		case kTurnOver: {
			TurningOverMotion();
			if (actual_robot_state_->is_robot_lose_control) {
				global_.state_initial_time = abs_run_time_;
				global_.state_posture_recovery = 0;
				global_.state = kAbnormalPostureRecovery;
			}
			break;
		}
	}

	if (actual_robot_state_->is_robot_lose_control && global_.state != kLoseControlProcess && global_.state != kWaitForStanding && global_.state != kAbnormalPostureRecovery) {
		global_.state_initial_time = abs_run_time_;
		FourLegsGetNowPosition();
		global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
		cout << "[Warning] Robot is lose control, Switch to position control mode, wait a moment to press [STAND UP BUTTON] to let robot lose power!!!" << endl;
		global_.state = kLoseControlProcess;
	}

	OutputProcess();
}

void BasicMotion::InitialStateRecord()
{
	if (!global_.is_switch_state_record) {
		is_robot_switch_state_ = false;
		global_.state_initial_time = abs_run_time_;
		global_.is_switch_state_record = true;
		if(global_.is_first_start_finish){
			global_.state = kVirtualForceStanding;
			SwitchToFourLegsStandRecord();
			if (global_.stand_up_down_flag_record == control_commands_.stand_up_down_flag) {
				global_.is_app_start_action = false;
			}
			global_.is_switch_to_force_finished = false;
			global_.switch_to_force_control_time = 1;
			cout << "Switching to force control mode, please wait..." << endl;
		} else {
			global_.state = kWaitForStanding;
			global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
      		global_.zeroposition_flag_record = control_commands_.zeroposition_flag;
			global_.turn_over_flag_record = control_commands_.turn_over_falg;			global_.app_start_stop_flag_record = control_commands_.app_start_stop_flag;
			global_.is_first_start_finish = true;
		}
		if (actual_robot_state_->is_robot_lose_control) {
			FourLegsGetNowPosition();
			global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
			cout << "[Warning] Robot is lose control, Switch to position control mode, wait a moment to press [STAND UP BUTTON] to let robot lose power!!!" << endl;
			global_.state = kLoseControlProcess;
		}
	}
}

void BasicMotion::OutputProcess()
{
	goal_robot_state_->forward_vel = 0;
	goal_robot_state_->side_vel = 0;
	if (global_.state == kWaitForStanding || global_.state == kPrepareMotion) {
		goal_robot_state_->body_height = body_attribute_.min_body_height;
	} else if (global_.state == kStandUpMotion || global_.state == kStandDownMotion) {
		goal_robot_state_->body_height = -fl_->goal_pos_[2];
	}
	if (global_.state != kVirtualForceStanding && global_.state != kRaiseHead) {
		goal_robot_state_->roll_posture = gyro_data_.roll;
		goal_robot_state_->pitch_posture = gyro_data_.pitch;
		goal_robot_state_->yaw_posture = gyro_data_.yaw;
	}

	actual_robot_state_->forward_vel = 0;
	actual_robot_state_->side_vel = 0;
	if (global_.state == kWaitForStanding || global_.state == kPrepareMotion) {
		actual_robot_state_->body_height = body_attribute_.min_body_height;
	} else {
		actual_robot_state_->body_height = (fl_actual_pos_[2] + fr_actual_pos_[2] + hl_actual_pos_[2] + hr_actual_pos_[2]) / 4;
	}
	actual_robot_state_->roll_posture = gyro_data_.roll;
	actual_robot_state_->pitch_posture = gyro_data_.pitch;
	actual_robot_state_->yaw_posture = gyro_data_.yaw;

	if (is_robot_switch_state_ || is_robot_switch_to_dance_ || is_robot_switch_to_pronk_ || is_robot_switch_to_jump_) {
		global_.is_switch_state_record = false;
	}
	robot_state_upload.robot_basic_state = global_.state;
	//cout<<"robot_state_upload.robot_basic_state = "<<robot_state_upload.robot_basic_state<<endl;
}

void BasicMotion::FourLegsAngleRecord()
{
	global_.fl_initial_angle = fl_->angle_;
	global_.fr_initial_angle = fr_->angle_;
	global_.hl_initial_angle = hl_->angle_;
	global_.hr_initial_angle = hr_->angle_;
}

void BasicMotion::FourLegsSwingToPositionAnglePlan(const Vec3 &goal_xyz_right, const double &total_time)
{
	Vec3 goal_xyz_left, goal_vel;
	goal_xyz_left << goal_xyz_right[0], -goal_xyz_right[1], goal_xyz_right[2];
	goal_vel << 0, 0, 0;
	fl_->SwingToPositionAnglePlan(global_.fl_initial_angle, goal_xyz_left, total_time, abs_run_time_ - global_.state_initial_time, "FL", 0);
	fr_->SwingToPositionAnglePlan(global_.fr_initial_angle, goal_xyz_right, total_time, abs_run_time_ - global_.state_initial_time, "FR", 0);
	hl_->SwingToPositionAnglePlan(global_.hl_initial_angle, goal_xyz_left, total_time, abs_run_time_ - global_.state_initial_time, "HL", 0);
	hr_->SwingToPositionAnglePlan(global_.hr_initial_angle, goal_xyz_right, total_time, abs_run_time_ - global_.state_initial_time, "HR", 0);
}

void BasicMotion::FourLegsSwingToPositionPathPlan(const Vec3 &goal_xyz_right, const double &total_time)
{
	Vec3 goal_xyz_left, goal_vel;
	Vec3 goal_xyz_right_(goal_xyz_right);
	goal_xyz_left << goal_xyz_right[0], -goal_xyz_right[1], goal_xyz_right[2];
	goal_vel << 0, 0, 0;
//	goal_xyz_left[0] = 0.04;
	SwingToPositionPathPlan(global_.fl_xyz, global_.fl_xyz_vel, goal_xyz_left, goal_vel, abs_run_time_ - global_.state_initial_time, total_time, "FL", fl_);
//	goal_xyz_right_[0] = -0.04;
	SwingToPositionPathPlan(global_.fr_xyz, global_.fr_xyz_vel, goal_xyz_right_, goal_vel, abs_run_time_ - global_.state_initial_time, total_time, "FR", fr_);
//	goal_xyz_left[0] = -0.04;
	SwingToPositionPathPlan(global_.hl_xyz, global_.hl_xyz_vel, goal_xyz_left, goal_vel, abs_run_time_ - global_.state_initial_time, total_time, "HL", hl_);
//	goal_xyz_right_[0] = 0.04;
	SwingToPositionPathPlan(global_.hr_xyz, global_.hr_xyz_vel, goal_xyz_right_, goal_vel, abs_run_time_ - global_.state_initial_time, total_time, "HR", hr_);
}

void BasicMotion::FourLegsKeepPosition(const Vec3 &goal_pos_right)
{
	Vec3 goal_pos_leg;
	Vec3 goal_pos_right_(goal_pos_right);
	goal_pos_leg << goal_pos_right[0], -goal_pos_right[1], goal_pos_right[2];

	fl_->SwingToPositionStandUpAndDown(goal_pos_leg, goal_pos_leg, goal_pos_leg, "FL");
	hl_->SwingToPositionStandUpAndDown(goal_pos_leg, goal_pos_leg, goal_pos_leg, "HL");
	hr_->SwingToPositionStandUpAndDown(goal_pos_right_, goal_pos_right_, goal_pos_right_, "HR");
	fr_->SwingToPositionStandUpAndDown(goal_pos_right_, goal_pos_right_, goal_pos_right_, "FR");
}

void BasicMotion::FourLegsGetNowPosition()
{
	GetNowPosition(fl_, &global_.fl_xyz, &global_.fl_xyz_vel);
	GetNowPosition(fr_, &global_.fr_xyz, &global_.fr_xyz_vel);
	GetNowPosition(hl_, &global_.hl_xyz, &global_.hl_xyz_vel);
	GetNowPosition(hr_, &global_.hr_xyz, &global_.hr_xyz_vel);
}

void BasicMotion::GetNowPosition(LegControl *leg, Vec3 *xyz, Vec3 *xyz_vel)
{
	Vec3 calculate_xyz, calculate_vel;

	leg->ForwardKinematic(leg->angle_, leg->velocity_, &calculate_xyz, &calculate_vel);

	for (int i = 0; i < 3; ++i) {
		(*xyz)[i] = calculate_xyz[i];
		(*xyz_vel)[i] = 0;
	}
}

void BasicMotion::SwingToPositionPathPlan(const Vec3 &init_xyz, const Vec3 &init_xyz_vel, const Vec3 &goal_xyz, const Vec3 &goal_xyz_vel, const double &run_time, const double &total_time, const string &side, LegControl *leg)
{
	double sub_goal_pos[3];
	double sub_goal_pos_next[3];
	double sub_goal_pos_next2[3];

	for (int i = 0; i < 3; ++i) {
		CubicSpline(init_xyz[i], init_xyz_vel[i], goal_xyz[i], goal_xyz_vel[i],
			abs_run_time_ - global_.state_initial_time, cycle_time_, total_time, sub_goal_pos[i], sub_goal_pos_next[i], sub_goal_pos_next2[i]);
	}

	leg->goal_pos_ << sub_goal_pos[0], sub_goal_pos[1], sub_goal_pos[2];
	leg->goal_pos_next_ << sub_goal_pos_next[0], sub_goal_pos_next[1], sub_goal_pos_next[2];
	leg->goal_pos_next2_ << sub_goal_pos_next2[0], sub_goal_pos_next2[1], sub_goal_pos_next2[2];

	leg->SwingToPosition(leg->goal_pos_, leg->goal_pos_next_, leg->goal_pos_next2_, 0, side);
}

void BasicMotion::SwitchToFourLegsStandRecord()
{
	//used for four leg virtual force stand
	global_.cent_angle[0] = gyro_data_.roll;
	global_.cent_angle[1] = gyro_data_.pitch;
	global_.cent_angle[2] = gyro_data_.yaw;
	global_.cent_xyz[0] = (fl_actual_pos_[0] + fr_actual_pos_[0] + hl_actual_pos_[0] + hr_actual_pos_[0]) / 4;
	global_.cent_xyz[1] = (fl_actual_pos_[1] + fr_actual_pos_[1] + hl_actual_pos_[1] + hr_actual_pos_[1]) / 4;
	global_.cent_xyz[2] = (fl_actual_pos_[2] + fr_actual_pos_[2] + hl_actual_pos_[2] + hr_actual_pos_[2]) / 4;
	if( global_.state == kGestureInteractionMotion ||  global_.state == kVoiceInteractionMotion) {
				global_.goal_yaw_angle = 0;
	} else {
		global_.goal_yaw_angle = gyro_data_.yaw;
		//global_.goal_yaw_angle = theta;
	}

	global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
	global_.raise_head_flag_record = control_commands_.raise_head_flag;
	global_.start_pronk_flag_record = control_commands_.start_pronk_flag;
	global_.turn_over_flag_record = control_commands_.turn_over_falg;
	global_.backflip_flag_record = control_commands_.start_backflip_flag;
	global_.prepare_backflip_flag_record = control_commands_.is_backflip_mode;

}

void BasicMotion::GetLegPositionGlobalBody(const string side, LegControl *leg, Vec3 *xyz, Vec3 *xyz_vel)
{
	Vec3 transform_xyz;
	if (side == "FL") {
		transform_xyz << body_attribute_.body_length_x, body_attribute_.body_length_y, 0;
	} else if (side == "FR") {
		transform_xyz << body_attribute_.body_length_x, -body_attribute_.body_length_y, 0;
	} else if (side == "HL") {
		transform_xyz << -body_attribute_.body_length_x, body_attribute_.body_length_y, 0;
	} else if (side == "HR") {
		transform_xyz << -body_attribute_.body_length_x, -body_attribute_.body_length_y, 0;
	}
	leg->ForwardKinematic(leg->angle_, leg->velocity_, xyz, xyz_vel);
	*xyz = transform_xyz + *xyz;  //transform the coordinate system between shoulder and pelma.
	*xyz_vel = RolPitYawToRM(gyro_data_, 0) * (*xyz_vel) + DerivRolPitYawToRM(gyro_data_, 0) * (*xyz);
	*xyz = RolPitYawToRM(gyro_data_, 0) * (*xyz);
}

void BasicMotion::GetLegPositionGlobal(const Vec3 &transform_xyz, LegControl *leg, Vec3 *xyz, Vec3 *xyz_vel)
{
	GetLegPositionLocal(transform_xyz, leg, xyz, xyz_vel);
	*xyz_vel = RolPitYawToRM(gyro_data_, 0) * (*xyz_vel) + DerivRolPitYawToRM(gyro_data_, 0) * (*xyz);
	*xyz = RolPitYawToRM(gyro_data_, 0) * (*xyz);
}

void BasicMotion::GetLegPositionLocal(const Vec3 &transform_xyz, LegControl *leg, Vec3 *xyz, Vec3 *xyz_vel)
{
	leg->ForwardKinematic(leg->angle_, leg->velocity_, xyz, xyz_vel);
	*xyz = transform_xyz - *xyz;  //transform the coordinate system between shoulder and pelma.
	*xyz_vel = -*xyz_vel;
}

void BasicMotion::ActualLegPosCalculation()
{
	Vec3 transform_xyz;
	transform_xyz << 0, 0, 0;
	GetLegPositionGlobal(transform_xyz, fl_, &fl_actual_pos_, &fl_actual_vel_);
	GetLegPositionGlobal(transform_xyz, fr_, &fr_actual_pos_, &fr_actual_vel_);
	GetLegPositionGlobal(transform_xyz, hl_, &hl_actual_pos_, &hl_actual_vel_);
	GetLegPositionGlobal(transform_xyz, hr_, &hr_actual_pos_, &hr_actual_vel_);

	GetLegPositionGlobalBody("FL", fl_, &fl_pos_body_, &fl_vel_body_);
	GetLegPositionGlobalBody("FR", fr_, &fr_pos_body_, &fr_vel_body_);
	GetLegPositionGlobalBody("HL", hl_, &hl_pos_body_, &hl_vel_body_);
	GetLegPositionGlobalBody("HR", hr_, &hr_pos_body_, &hr_vel_body_);
}

void BasicMotion::CommandFollow(const double goal_x_pos, const double goal_y_pos, const double goal_z_pos, const double goal_roll_angle, const double goal_pitch_angle, const double goal_yaw_angle)
{
	double Fx, Fy, Fz;
	double goal_height, goal_height_vel;
	double goal_x,  goal_x_vel;
	double goal_y,  goal_y_vel;
	double goal_roll, torque_roll, goal_rol_vel;
	double goal_pitch,  torque_pitch, goal_pitch_vel;
	double goal_yaw,torque_yaw, goal_yaw_vel;
	double theta = 0; double dtheta = 0;
	Vec3 average_xyz, average_xyz_vel;
	average_xyz = (fl_actual_pos_ + fr_actual_pos_ + hl_actual_pos_ + hr_actual_pos_) / 4;
	average_xyz_vel = (fl_actual_vel_ + fr_actual_vel_ + hl_actual_vel_ + hr_actual_vel_) / 4;

	goal_x = goal_x_pos; goal_y = goal_y_pos; goal_height = goal_z_pos;
	goal_x_vel = 0; goal_y_vel = 0; goal_height_vel = 0;
	goal_roll = goal_roll_angle;
	goal_pitch = goal_pitch_angle;
	goal_yaw = goal_yaw_angle;
	goal_rol_vel = 0; goal_pitch_vel = 0; goal_yaw_vel = 0;

	GetStandingStateYawAngle(theta,dtheta);

	Fx = (1*default_control_parameters_.stand.fx_kp * (goal_x - average_xyz[0]) + default_control_parameters_.stand.fx_kd * (goal_x_vel - average_xyz_vel[0]));
	Fy = (1*default_control_parameters_.stand.fy_kp * (goal_y - average_xyz[1]) + default_control_parameters_.stand.fy_kd * (goal_y_vel - average_xyz_vel[1]));
	Fz = (1.1*default_control_parameters_.stand.fz_kp* (goal_height - average_xyz[2]) + default_control_parameters_.stand.fz_kd * (goal_height_vel - average_xyz_vel[2])) + body_attribute_.mass * kGravity;

	torque_roll  = 0.7 * (1 * default_control_parameters_.stand.roll_kp * AbsLim(5*kDegree2Radian,goal_roll - gyro_data_.roll) +1* default_control_parameters_.stand.roll_kd* (goal_rol_vel - gyro_data_.rol_vel));
	torque_pitch = 0.7 * (1 * default_control_parameters_.stand.pitch_kp * AbsLim(5*kDegree2Radian, goal_pitch - gyro_data_.pitch) +1* default_control_parameters_.stand.pitch_kd * (goal_pitch_vel - gyro_data_.pit_vel));
	torque_yaw   = 0.7 * (1 * default_control_parameters_.stand.yaw_kp * AbsLim(5*kDegree2Radian, goal_yaw - theta) + 1*default_control_parameters_.stand.yaw_kd * (goal_yaw_vel - dtheta));//d=600

	torque_roll = min(max(-6.0,torque_roll),6.0);
	torque_pitch = min(max(-10.0,torque_pitch),10.0);
	torque_yaw = min(max(-6.0,torque_yaw),6.0);
	
	Fz = min(max(0.7*body_attribute_.mass * kGravity,Fz),1.8*body_attribute_.mass * kGravity);
	Fx = min(max(-0.6*Fz,Fx),0.6*Fz);
	Fy = min(max(-0.6*Fz,Fy),0.6*Fz);

	Vec3 virtual_force_fl, virtual_force_fr, virtual_force_hl, virtual_force_hr;

	ForceTorqueSolveFLFRHLHR(Fx, Fy, Fz, torque_roll, torque_pitch, torque_yaw, fl_actual_pos_, fr_actual_pos_, hl_actual_pos_, hr_actual_pos_,
							 &virtual_force_fl, &virtual_force_fr, &virtual_force_hl, &virtual_force_hr);

	fl_->VirtualForceToTorqueOutput(virtual_force_fl);
	fr_->VirtualForceToTorqueOutput(virtual_force_fr);
	hl_->VirtualForceToTorqueOutput(virtual_force_hl);
	hr_->VirtualForceToTorqueOutput(virtual_force_hr);
}

void BasicMotion::GetStandingStateYawAngle(double& yaw,double& yaw_vel){
	double y = ((fl_actual_pos_[1] + fr_actual_pos_[1])/2.0 - (hl_actual_pos_[1] + hr_actual_pos_[1])/2.0);
	double x = ((fl_actual_pos_[0] + fr_actual_pos_[0])/2.0 - (hl_actual_pos_[0] + hr_actual_pos_[0])/2.0) + 2*body_attribute_.body_length_x;
	if(x != 0){
		double y_x = y/x;
		yaw = atan(y_x);
		double dy = (fl_actual_vel_[1] + fr_actual_vel_[1])/2.0 - (hl_actual_vel_[1] + hr_actual_vel_[1])/2.0;
		double dx = (fl_actual_vel_[0] + fr_actual_vel_[0])/2.0 - (hl_actual_vel_[0] + hr_actual_vel_[0])/2.0;
		yaw_vel = 1.0/(1.0+y_x*y_x)*(dy*x-dx*y)/(x*x);
	}else{
		yaw = 0;
		yaw_vel = 0;
		cout<<"Standing Yaw Angle Error!"<<endl;
	}
//	filter_input[2] = yaw_vel;
//	LowPassFilter(filter_input[0], filter_input[1], filter_input[2],filter_output[0], filter_output[1], yaw_vel);
//	cout<<"yaw_vel:"<<yaw_vel<<endl;
	yaw_vel =  gyro_data_.yaw_vel;
	return;
}

void BasicMotion::ForceTorqueSolveFLFRHLHR(const double &Fx, const double &Fy, const double &Fz, const double &torque_roll, const double &torque_pitch, const double &torque_yaw,
											 const Vec3 &fl_xyz, const Vec3 &fr_xyz, const Vec3 &hl_xyz, const Vec3 &hr_xyz, Vec3 *F1, Vec3 *F2, Vec3 *F3, Vec3 *F4) {
	Matrix< double, 6, 12> A;
	Matrix< double, 12, 6> A_trans;
	Matrix< double, 12, 1> F_solution;
	Matrix< double, 6, 1> F_matrix;
	Matrix< double, 6, 6> A_trans_A;
	double lx = 2 * body_attribute_.body_length_x;
	double ly = 2 * body_attribute_.body_length_y;

	F_matrix << Fx, Fy, Fz, torque_roll, torque_pitch, torque_yaw;

	A << 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0,
		0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
		0, fl_xyz[2], ly / 2 - fl_xyz[1], 0, fr_xyz[2], -ly / 2 - fr_xyz[1], 0, hl_xyz[2], ly / 2 - hl_xyz[1], 0, hr_xyz[2], -ly / 2 - hr_xyz[1],
		-fl_xyz[2], 0, -lx / 2 + fl_xyz[0], -fr_xyz[2], 0, -lx / 2 + fr_xyz[0], -hl_xyz[2], 0, lx / 2 + hl_xyz[0], -hr_xyz[2], 0, lx / 2 + hr_xyz[0],
		-ly / 2 + fl_xyz[1], lx / 2 - fl_xyz[0], 0, ly / 2 + fr_xyz[1], lx / 2 - fr_xyz[0], 0, -ly / 2 + hl_xyz[1], -lx / 2 - hl_xyz[0], 0, ly / 2 + hr_xyz[1], -lx / 2 - hr_xyz[0], 0;

	A_trans = A.adjoint();
	A_trans_A = A * A_trans;
	F_solution = A_trans * (A_trans_A.inverse())*F_matrix;

	(*F1)[0] = F_solution(0);
	(*F1)[1] = F_solution(1);
	(*F1)[2] = F_solution(2);
	(*F2)[0] = F_solution(3);
	(*F2)[1] = F_solution(4);
	(*F2)[2] = F_solution(5);
	(*F3)[0] = F_solution(6);
	(*F3)[1] = F_solution(7);
	(*F3)[2] = F_solution(8);
	(*F4)[0] = F_solution(9);
	(*F4)[1] = F_solution(10);
	(*F4)[2] = F_solution(11);
}

void BasicMotion::AbnormalPostureRecovery()
{
	switch (global_.state_posture_recovery) {
		case 0: {
			if (abs_run_time_ < global_.state_initial_time + 0.3) {
				global_.stand_up_down_flag_record = control_commands_.stand_up_down_flag;
			} else {
				if ((global_.stand_up_down_flag_record != control_commands_.stand_up_down_flag)) {
					cout << "Robot is recovering posture..." << endl;
					global_.state_initial_time = abs_run_time_;
					FourLegsAngleRecord();
					global_.state_posture_recovery = 1;
					global_.pos_recovery_to_stand_up_flag = 1;
					global_.turn_over_to_pos_recovery_flag = 0;
				}
			}
			break;
		}

		case 1: {
			if (abs(gyro_data_.roll) <= 60 * kDegree2Radian && abs(gyro_data_.rol_vel) <= 0.2) {
				cout << "Robot is waitting to stand up!!!" << endl;
				actual_robot_state_->is_robot_lose_control = false;
			} else {
				if (control_commands_.recover_direction_flag == false) {
					global_.turn_over_flag = 1;
				} else if(control_commands_.recover_direction_flag == true){
					global_.turn_over_flag = 0;
				}
				Vec3 goal_pos_right;
				Vec3 goal_angle_fl, goal_angle_fr, goal_angle_hl, goal_angle_hr;
				goal_pos_right << 0, -fr_->link_.link_length[0], -body_attribute_.min_body_height;
				if (abs_run_time_ <= global_.state_initial_time + kInitialPoseTime) {
					goal_angle_fl << fl_->angle_[0], fl_->angle_[1], 150 * kDegree2Radian;
					goal_angle_fr << fr_->angle_[0], fr_->angle_[1], 150 * kDegree2Radian;
					goal_angle_hr << hr_->angle_[0], hr_->angle_[1], 150 * kDegree2Radian;
					goal_angle_hl << hl_->angle_[0], hl_->angle_[1], 150 * kDegree2Radian;
					fl_->SwingToPositionAnglePlan(global_.fl_initial_angle, goal_angle_fl, kInitialPoseTime, abs_run_time_ - global_.state_initial_time, "FL", 1);
					fr_->SwingToPositionAnglePlan(global_.fr_initial_angle, goal_angle_fr, kInitialPoseTime, abs_run_time_ - global_.state_initial_time, "FR", 1);
					hl_->SwingToPositionAnglePlan(global_.hl_initial_angle, goal_angle_hl, kInitialPoseTime, abs_run_time_ - global_.state_initial_time, "HL", 1);
					hr_->SwingToPositionAnglePlan(global_.hr_initial_angle, goal_angle_hr, kInitialPoseTime, abs_run_time_ - global_.state_initial_time, "HR", 1);
					//FourLegsSwingToPositionAnglePlan(goal_pos_right, kInitialPoseTime);
				} else {
			//		FourLegsKeepPosition(goal_pos_right);
					FourLegsAngleRecord();
					if (control_commands_.recover_direction_flag == false) {
						cout<<"Recover Left!!!"<<endl;
					} else if(control_commands_.recover_direction_flag == true){
						cout<<"Recover Right!!!"<<endl;
					}
					global_.state_initial_time = abs_run_time_;
					global_.state_posture_recovery = 2;
				}
			}
			break;
		}

		case 2: {
			if (robot_id_ == "JY-P"){
				global_.turn_over_time=2;
			}else if(robot_id_ == "JY-L"){
				global_.turn_over_time=1.7;
			}else if(robot_id_ == "JY-M"){
				global_.turn_over_time=1;
			}else if(robot_id_ == "JY-S"){
				global_.turn_over_time=0.4;
			}else{
				global_.turn_over_time=1.5;
			}
			//double turn_over_time = 1;
			Vec3 goal_angle_fl, goal_angle_fr, goal_angle_hl, goal_angle_hr;
			//choose target through the flag
			Vec3 goal_pos_right;
			goal_pos_right << 0, -fr_->link_.link_length[0], -body_attribute_.min_body_height;
			if (abs_run_time_ <= global_.state_initial_time + global_.turn_over_time) {
				FourLegsSwingToPositionAnglePlan(goal_pos_right, global_.turn_over_time);
			} else {
				global_.state_initial_time = abs_run_time_;
				FourLegsAngleRecord();
				global_.state_posture_recovery = 3;
			}
			break;
		}
		case 3: {
			//double turn_over_time = 1;
			Vec3 goal_angle_fl, goal_angle_fr, goal_angle_hl, goal_angle_hr;
			//choose target through the flag
			// global_.turn_over_time = 0.5;
			if (global_.turn_over_flag == 1) {
				goal_angle_fl << -8 * kDegree2Radian, -160 * kDegree2Radian, 150 * kDegree2Radian;
				goal_angle_fr << 11 * kDegree2Radian, global_.fr_initial_angle[1], global_.fr_initial_angle[2];
				goal_angle_hr << 11 * kDegree2Radian, global_.hr_initial_angle[1] , global_.hr_initial_angle[2];
				goal_angle_hl << -8 * kDegree2Radian, -160 * kDegree2Radian, 150 * kDegree2Radian;
			} else {
				goal_angle_fl << -11 * kDegree2Radian, global_.fl_initial_angle[1], global_.fl_initial_angle[2];
				goal_angle_fr << 8 * kDegree2Radian, -160 * kDegree2Radian, 150 * kDegree2Radian;
				goal_angle_hr << 8 * kDegree2Radian, -160 * kDegree2Radian, 150 * kDegree2Radian;
				goal_angle_hl << -11 * kDegree2Radian, global_.hl_initial_angle[1], global_.hl_initial_angle[2];

			}
			if (abs_run_time_ <= global_.state_initial_time + global_.turn_over_time) {
				fl_->SwingToPositionAnglePlan(global_.fl_initial_angle, goal_angle_fl, global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "FL", 1);
				fr_->SwingToPositionAnglePlan(global_.fr_initial_angle, goal_angle_fr, global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "FR", 1);
				hl_->SwingToPositionAnglePlan(global_.hl_initial_angle, goal_angle_hl, global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "HL", 1);
				hr_->SwingToPositionAnglePlan(global_.hr_initial_angle, goal_angle_hr, global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "HR", 1);
			} else {
				global_.state_initial_time = abs_run_time_;
				FourLegsAngleRecord();
				global_.state_posture_recovery = 4;
			}
			break;
		}

		case 4: {
			//double turn_over_time = 1;
			//global_.turn_over_time = 0.5;
			Vec3 goal_angle_fl, goal_angle_fr, goal_angle_hl, goal_angle_hr;
			if (global_.turn_over_flag == 1) {
				goal_angle_fl << -22*kDegree2Radian, -180 * kDegree2Radian, 30 * kDegree2Radian;
				goal_angle_fr << 22 * kDegree2Radian, -100 * kDegree2Radian, global_.fr_initial_angle[2];
				goal_angle_hr << 22 * kDegree2Radian, -100 * kDegree2Radian, global_.hr_initial_angle[2];
				goal_angle_hl  << -22*kDegree2Radian, -180 * kDegree2Radian, 30 * kDegree2Radian;
			} else {
				goal_angle_fl << -22 * kDegree2Radian, -100 * kDegree2Radian, global_.fl_initial_angle[2];
				goal_angle_fr << 22 * kDegree2Radian, -180 * kDegree2Radian, 30 * kDegree2Radian;
				goal_angle_hr << 22 * kDegree2Radian, -180 * kDegree2Radian, 30 * kDegree2Radian;
				goal_angle_hl << -22 * kDegree2Radian, -100 * kDegree2Radian, global_.hl_initial_angle[2];
			}
			if (abs_run_time_ <= global_.state_initial_time + global_.turn_over_time) {
				fl_->SwingToPositionAnglePlan(global_.fl_initial_angle, goal_angle_fl, global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "FL", 1);
				fr_->SwingToPositionAnglePlan(global_.fr_initial_angle, goal_angle_fr, global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "FR", 1);
				hl_->SwingToPositionAnglePlan(global_.hl_initial_angle, goal_angle_hl, global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "HL", 1);
				hr_->SwingToPositionAnglePlan(global_.hr_initial_angle, goal_angle_hr, global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "HR", 1);
			} else {
				FourLegsAngleRecord();
				global_.state_initial_time = abs_run_time_;
				global_.state_posture_recovery = 5;
			}
			break;
		}

		case 5: {
			//double turn_over_time = 1;
			double keep_time=0.2;//给一小段缓冲时间
			global_.turn_over_time = 1;
			Vec3 goal_angle_fl, goal_angle_fr, goal_angle_hl, goal_angle_hr;
			if (global_.turn_over_flag == 1) {
				goal_angle_fl << -15 * kDegree2Radian, -102 * kDegree2Radian, 130 * kDegree2Radian;
				goal_angle_fr << -25 * kDegree2Radian, -110 * kDegree2Radian, global_.fr_initial_angle[2];
				goal_angle_hr << -25 * kDegree2Radian, -110 * kDegree2Radian, global_.hr_initial_angle[2];
				goal_angle_hl << -15 * kDegree2Radian, -102 * kDegree2Radian,130 * kDegree2Radian;
			} else {
				goal_angle_fl << 25 * kDegree2Radian, -110 * kDegree2Radian, global_.fl_initial_angle[2];
				goal_angle_fr << 15 * kDegree2Radian, -102 * kDegree2Radian, 130 * kDegree2Radian;
				goal_angle_hr << 15 * kDegree2Radian, -102 * kDegree2Radian, 130 * kDegree2Radian;
				goal_angle_hl << 25 * kDegree2Radian, -110 * kDegree2Radian, global_.hl_initial_angle[2];
			}
			if (abs_run_time_ <= global_.state_initial_time + 0.5*global_.turn_over_time) {
				fl_->SwingToPositionAnglePlan(global_.fl_initial_angle, goal_angle_fl, 0.5*global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "FL", 1);
				fr_->SwingToPositionAnglePlan(global_.fr_initial_angle, goal_angle_fr, 0.5*global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "FR", 1);
				hl_->SwingToPositionAnglePlan(global_.hl_initial_angle, goal_angle_hl, 0.5*global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "HL", 1);
				hr_->SwingToPositionAnglePlan(global_.hr_initial_angle, goal_angle_hr, 0.5*global_.turn_over_time, abs_run_time_ - global_.state_initial_time, "HR", 1);
			} else {
				fl_->SwingToPositionAnglePlan(global_.fl_initial_angle, goal_angle_fl, keep_time, abs_run_time_ - global_.state_initial_time, "FL", 2);
				fr_->SwingToPositionAnglePlan(global_.fr_initial_angle, goal_angle_fr, keep_time, abs_run_time_ - global_.state_initial_time, "FR", 2);
				hl_->SwingToPositionAnglePlan(global_.hl_initial_angle, goal_angle_hl, keep_time, abs_run_time_ - global_.state_initial_time, "HL", 2);
				hr_->SwingToPositionAnglePlan(global_.hr_initial_angle, goal_angle_hr, keep_time, abs_run_time_ - global_.state_initial_time, "HR", 2);
				if (abs(gyro_data_.roll) <= 30 * kDegree2Radian && abs(gyro_data_.rol_vel) <= 0.2 && abs_run_time_ > global_.state_initial_time + global_.turn_over_time + 0.01) {
					FourLegsAngleRecord();
					global_.state_initial_time = abs_run_time_;
					global_.state_posture_recovery = 6;
				} else if (abs(gyro_data_.roll) > 30 * kDegree2Radian && abs_run_time_ >= global_.state_initial_time + global_.turn_over_time + 3) {
					cout << "[Error] Robot recovering posture failed!!!" << endl;
					global_.state_initial_time = abs_run_time_;
					global_.state_posture_recovery = 0;
				}
			}
			break;
		}

		case 6: {
			double turn_over_time = 0.5;//复位
			Vec3 goal_angle_fl, goal_angle_fr, goal_angle_hl, goal_angle_hr;
			if (global_.turn_over_flag == 1) {
				goal_angle_fl << 0 * kDegree2Radian,-80 * kDegree2Radian, 160 * kDegree2Radian;//= global_->fl_initial_angle;
				goal_angle_fr << 0 * kDegree2Radian,-80 * kDegree2Radian, 160 * kDegree2Radian;
				goal_angle_hr << 0 * kDegree2Radian,-80 * kDegree2Radian, 160 * kDegree2Radian;
				goal_angle_hl << 0 * kDegree2Radian,-80 * kDegree2Radian, 160 * kDegree2Radian;
			} else {
				goal_angle_fl << 0 * kDegree2Radian,-80 * kDegree2Radian, 160 * kDegree2Radian;//= global_->fl_initial_angle;
				goal_angle_fr << 0 * kDegree2Radian,-80 * kDegree2Radian, 160 * kDegree2Radian;
				goal_angle_hr << 0 * kDegree2Radian,-80 * kDegree2Radian, 160 * kDegree2Radian;
				goal_angle_hl << 0 * kDegree2Radian,-80 * kDegree2Radian, 160 * kDegree2Radian;
			}
			if (abs_run_time_ <= global_.state_initial_time + turn_over_time) {
				fl_->SwingToPositionAnglePlan(global_.fl_initial_angle, goal_angle_fl, turn_over_time, abs_run_time_ - global_.state_initial_time, "FL", 1);
				fr_->SwingToPositionAnglePlan(global_.fr_initial_angle, goal_angle_fr, turn_over_time, abs_run_time_ - global_.state_initial_time, "FR", 1);
				hl_->SwingToPositionAnglePlan(global_.hl_initial_angle, goal_angle_hl, turn_over_time, abs_run_time_ - global_.state_initial_time, "HL", 1);
				hr_->SwingToPositionAnglePlan(global_.hr_initial_angle, goal_angle_hr, turn_over_time, abs_run_time_ - global_.state_initial_time, "HR", 1);
			} else {
				cout << "Robot recovering posture succeed, waitting to stand up!!!" << endl;
				actual_robot_state_->is_robot_lose_control = false;
			}
			break;
		}


		default:
			break;
	}
//		cout<<"recovery state:"<<global_.state_posture_recovery<<endl;
}

void BasicMotion::TurningOverMotion()
{
	switch (global_.state_turn_over) {
		case 0: {
			Vec3 goal_pos_right;
			goal_pos_right << 0, -fr_->link_.link_length[0], -body_attribute_.min_body_height;
			if (abs_run_time_ <= global_.state_initial_time + kInitialPoseTime) {
				FourLegsSwingToPositionAnglePlan(goal_pos_right, kInitialPoseTime);
			} else {
				FourLegsKeepPosition(goal_pos_right);
				FourLegsAngleRecord();
				global_.state_initial_time = abs_run_time_;
				global_.state_turn_over = 1;
			}
			break;
		}

		case 1: {
			double turn_over_time = 0.35;
			Vec3 goal_angle_fl, goal_angle_hl, goal_angle_fr, goal_angle_hr;
			goal_angle_fr << 24 * kDegree2Radian, -20 * kDegree2Radian, 40 * kDegree2Radian;
			goal_angle_hr << 24 * kDegree2Radian, -20 * kDegree2Radian, 40 * kDegree2Radian;
			goal_angle_fl << 24 * kDegree2Radian, global_.fl_initial_angle[1], global_.fl_initial_angle[2];
			goal_angle_hl << 24 * kDegree2Radian, global_.hl_initial_angle[1], global_.hl_initial_angle[2];

			if (abs_run_time_ <= global_.state_initial_time + turn_over_time) 
			{
				fl_->SwingToPositionAnglePlan(global_.fl_initial_angle, goal_angle_fl, turn_over_time, abs_run_time_ - global_.state_initial_time, "FL", 1);
				fr_->SwingToPositionAnglePlan(global_.fr_initial_angle, goal_angle_fr, turn_over_time, abs_run_time_ - global_.state_initial_time, "FR", 1);
				hl_->SwingToPositionAnglePlan(global_.hl_initial_angle, goal_angle_hl, turn_over_time, abs_run_time_ - global_.state_initial_time, "HL", 1);
				hr_->SwingToPositionAnglePlan(global_.hr_initial_angle, goal_angle_hr, turn_over_time, abs_run_time_ - global_.state_initial_time, "HR", 1);
			} else {
				FourLegsAngleRecord();
				global_.state_initial_time = abs_run_time_;
				global_.state_turn_over = 2;
			}

			break;
		}
			 

		// contract the leg
		case 2: {
			double contract_leg_time = 0.5;

			Vec3 goal_angle_fl, goal_angle_hl, goal_angle_fr, goal_angle_hr;
			goal_angle_fr << 24 * kDegree2Radian, -80 * kDegree2Radian, 160 * kDegree2Radian;
			goal_angle_hr << 24 * kDegree2Radian, -80 * kDegree2Radian, 160 * kDegree2Radian;
			goal_angle_fl << 24 * kDegree2Radian, -80 * kDegree2Radian, 160 * kDegree2Radian;
			goal_angle_hl << 24 * kDegree2Radian, -80 * kDegree2Radian, 160 * kDegree2Radian;

			if (abs_run_time_ <= global_.state_initial_time + contract_leg_time) 
			{
				fl_->SwingToPositionAnglePlan(global_.fl_initial_angle, goal_angle_fl, contract_leg_time, abs_run_time_ - global_.state_initial_time, "FL", 1);
				fr_->SwingToPositionAnglePlan(global_.fr_initial_angle, goal_angle_fr, contract_leg_time, abs_run_time_ - global_.state_initial_time, "FR", 1);
				hl_->SwingToPositionAnglePlan(global_.hl_initial_angle, goal_angle_hl, contract_leg_time, abs_run_time_ - global_.state_initial_time, "HL", 1);
				hr_->SwingToPositionAnglePlan(global_.hr_initial_angle, goal_angle_hr, contract_leg_time, abs_run_time_ - global_.state_initial_time, "HR", 1);
			} else {
				FourLegsAngleRecord();
				actual_robot_state_->is_robot_lose_control = true;
				global_.state = kAbnormalPostureRecovery;
				global_.state_posture_recovery = 0;
			}
			break;
		}
	}
}

