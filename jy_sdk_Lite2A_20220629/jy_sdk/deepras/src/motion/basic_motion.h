
#ifndef MOTION_BASIC_MOTION_H_
#define MOTION_BASIC_MOTION_H_

#include "basic_function/one_leg_control.h"

#include "parameters/parameters_share.h"
#include "parameters/parameters_basic_motion.h"
#include "parameters/performance_states.h"
#include <cmath>
#include <iostream>

using namespace basicmethod;

namespace basic_motion {
	class BasicMotion {
	public:
		BasicMotion(LegControl *fl,
			LegControl *fr,
			LegControl *hl,
			LegControl *hr,
			RobotState *goal_robot_state,
			RobotState *actual_robot_state);
		~BasicMotion();
		bool is_robot_switch_state_;
		bool is_robot_switch_to_dance_;
		bool is_robot_switch_to_pronk_;
		bool is_robot_switch_to_jump_;

		void ConstParameterInitial(const string &robot_id, const string &robot_type);
		void DataInputProcess(const CommandsBasicMotion &control_commands, const double &abs_run_time, const double &cycle_time, const DataGyro &gyro_data_process);
		void MotionProcess();
		string robot_id_;

	protected:
		void InitialStateRecord();
		void OutputProcess();

		void FourLegsAngleRecord();
		void FourLegsSwingToPositionAnglePlan(const Vec3 &goal_xyz_right, const double &total_time);
		void FourLegsSwingToPositionPathPlan(const Vec3 &goal_xyz_right, const double &total_time);
		void FourLegsKeepPosition(const Vec3 &goal_pos_right);
		void FourLegsGetNowPosition();
		void GetNowPosition(LegControl *leg, Vec3 *xyz, Vec3 *xyz_vel);
		void SwingToPositionPathPlan(const Vec3 &init_xyz, const Vec3 &init_xyz_vel, const Vec3 &goal_xyz, const Vec3 &goal_xyz_vel, const double &run_time, const double &total_time, const string &side, LegControl *leg);
		void SwitchToFourLegsStandRecord();
		void GetLegPositionGlobal(const Vec3 &transform_xyz, LegControl *leg, Vec3 *xyz_1, Vec3 *xyz_vel_1);
		void GetLegPositionLocal(const Vec3 &transform_xyz, LegControl *leg, Vec3 *xyz, Vec3 *xyz_vel);
		void GetLegPositionGlobalBody(const string side, LegControl *leg, Vec3 *xyz, Vec3 *xyz_vel);
		void VirtualForceDistribution(const double goal_x_pos, const double goal_y_pos, const double goal_z_pos, const double goal_roll_angle, const double goal_pitch_angle, const double goal_yaw_angle);
		void ActualLegPosCalculation();
		void ForceTorqueSolveFLFRHLHR(const double &Fx, const double &Fy, const double &Fz, const double &torque_roll, const double &torque_pitch, const double &torque_yaw,
			const Vec3 &fl_xyz, const Vec3 &fr_xyz, const Vec3 &hl_xyz, const Vec3 &hr_xyz, Vec3 *F1, Vec3 *F2, Vec3 *F3, Vec3 *F4);
		void ForceTorqueSolveFLFRHLHR2(const double &Fx, const double &Fy, const double &Fz, const double &torque_roll, const double &torque_pitch, const double &torque_yaw,
													 const Vec3 &fl_xyz, const Vec3 &fr_xyz, const Vec3 &hl_xyz, const Vec3 &hr_xyz, Vec3 *F1, Vec3 *F2, Vec3 *F3, Vec3 *F4);
		void CommandFollow(const double goal_x_pos, const double goal_y_pos, const double goal_z_pos, const double goal_roll_angle, const double goal_pitch_angle, const double goal_yaw_angle);
		void AbnormalPostureRecovery();
		void TurningOverMotion();
		void Backflip();
		void GetStandingStateYawAngle(double& yaw,double& yaw_vel);
		void TerrainEstimateByFourLegGlobal(Vec3 fl_global, Vec3 fr_global, Vec3 hl_global, Vec3 hr_global, double &pitch_estimate, double &roll_estimate);
		bool IsRobotCanVirtualForceStand();
		
		LegControl *fl_, *fr_, *hl_, *hr_;
		CommandsBasicMotion control_commands_;
		ControlParameter default_control_parameters_;
		bool is_support_dance_;
		bool is_support_pronk_;
		bool is_support_jump_;
		double virtual_force_PD_rate_;
		double raise_head_stand_height_;
		double virtual_force_yaw_kp_;
		DataGyro gyro_data_;
		static DataGlobal global_;
		BodyAttribute body_attribute_;
		double abs_run_time_; //unit: s
		double cycle_time_;//unit: s
		RobotState *actual_robot_state_;
		RobotState *goal_robot_state_;
		Vec3 fl_actual_pos_, fr_actual_pos_, hl_actual_pos_, hr_actual_pos_;
		Vec3 fl_actual_vel_, fr_actual_vel_, hl_actual_vel_, hr_actual_vel_;

		Vec3 fl_pos_body_, fr_pos_body_, hl_pos_body_, hr_pos_body_;
		Vec3 fl_vel_body_, fr_vel_body_, hl_vel_body_, hr_vel_body_;

		double filter_input[3];
		double filter_output[3];
	};  // BasicMotion
}
#endif  // MOTION_BASIC_MOTION_H_
