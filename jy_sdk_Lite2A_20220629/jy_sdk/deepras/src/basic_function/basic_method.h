
#ifndef BASIC_METHOD_H_
#define BASIC_METHOD_H_

#include "parameters/parameters_share.h"
#include <time.h> 


namespace basicmethod {
	Eigen::Matrix3d RolPitYawToRM(const DataGyro &gyro);
	Eigen::Matrix3d RolPitYawToRM(const DataGyro &gyro, const int &i);
	Eigen::Matrix3d RolPitYawToRM(const Vec3 &angles);
	Eigen::Matrix3d RolPitYawToRM(const Vec3 &angles, const int &i);
	Eigen::Matrix3d DerivRolPitYawToRM(const DataGyro &gyro);
	Eigen::Matrix3d DerivRolPitYawToRM(const DataGyro &gyro, const int &i);

	Vec3 VirtualForceCoordinateTransformFrom(const Vec3 &ori, const Vec3 &ang, const double &angle_hipy);
	Vec3 VirtualForceCoordinateTransformTo(const Vec3 &ori, const Vec3 &ang, const double &angle_hipy);

	int Sign(const double &i);
	void GetAbsMin(const double &ref, double &own);
	void LimitRange(const double &min, const double &max, double &own);
	void GetAbsMax(const double &ref, double &own);
	void GetAbsMin(const Vec3 &ref, Vec3 &own);
	void GetMin(const double &ref, double &own);
	void GetMax(const double &ref, double &own);
	double AbsLim(double bound, double a);

	//************ 非标准，待改
	void VectorToArrary(double *arr, Vec3 &vec, int cntb, int num);
	void VectorToArrary(int *arr, Vec3 &vec, int cntb, int num);
	void VectorToArrary(double *arr, Vec3 &vec, int num);
	void VectorToArrary(int *arr, Vec3 &vec, int num);
	void ArraryToVector(Vec3 &vec, double *arr, int cntb, int num);
	void ArraryToVector(Vec3 &vec, double *arr, int num);

	void CubicSpline(double init_pos, double init_vel, double goal_pos, double goal_vel, double run_time,
		 double cycle_time, double total_time, double &sub_goal_pos, double &sub_goal_pos_next);
	void CubicSpline(double init_pos, double init_vel, double goal_pos, double goal_vel, double run_time,
		             double cycle_time, double total_time, double &sub_goal_pos, double &sub_goal_pos_next, double &sub_goal_pos_next2);
	double CosineSpline(double init_pos, double goal_pos, double run_time, double total_time);
	double CosineSpline(double init_pos, double goal_pos, double run_time, double total_time, double p_dh);

	//************ 非标准，待改

	Eigen::Matrix3d RPY_to_RM(const Vec3 &RPYs);

	void AddToScope(const double data, const int num, const string var_name);
	void ScopeNamesOutput();
	void MotionConfigInitial(const string &robot_id, const string &robot_type, const MotionConfig &external_motion_config);
	bool RobotReachLocalPoint(const double &goal_x, const double &goal_y, const double &goal_theta, const double &vision_location_x, const double &vision_location_y, const double &vision_location_theta,
		const double &threshold_x, const double &threshold_y, const double &threshold_y_final, const double &threshold_theta, const double &high_x_vel, const double &low_x_vel, const double &abs_run_time, InnerControlCommands *inner_control_commands);
	void DataPreProcessing(const string &robot_id, const int &control_state_record, RobotState &actual_robot_state, const DataGyro &gyro_data_raw, const ControlCommands &control_commands_raw, const double &abs_run_time,
						   DataGyro *gyrp_data_process, InnerControlCommands *inner_control_commands, scope_output *scope, DataLegs *leg_output);
	void RobotStateOutput(const RobotState &goal_robot_state, const RobotState &actual_robot_state);
	void OutputDataProcessing(const Vec3 &fl_pos, const Vec3 &fl_vel, const Vec3 &fl_torque, const Vec3 &fr_pos, const Vec3 &fr_vel, const Vec3 &fr_torque, 
							  const Vec3 &hl_pos, const Vec3 &hl_vel, const Vec3 &hl_torque, const Vec3 &hr_pos, const Vec3 &hr_vel, const Vec3 &hr_torque,
							  const OneLegControlParameter &control_parameters, DataLegs *leg_output);
	void GyroDataProcess(const DataGyro &gyro_data_raw, DataGyro *gyrp_data_process);
	void IsRobotLoseControl(RobotState &actual_robot_state,RobotState &goal_robot_state,const DataGyro &gyrp_data_process);
	enum RobotStateMap {
		kBasicMotion =	    0,
		kTrot        =			1,
		kJumpTrot    =		  2,
		kStairTrot   =	    3,
		kBound       =	   	4,
		kPronk       =		  5,
		kPace        =			6,
		kDance       =		  7,
		kJump        =			8,
		kWalk        =      9,
		kTrotRun     =      10,
	};
	enum GestureValue {
		kGestureMoveForward =	1,
		kGestureMoveBackward =	2,
		kGestureStandUp =		3,
		kGestureStandDown =		4,
		kGestureStop =			5,
		kGestureTurnLeft =		6,
		kGestureTurnRight =		7,
		kGestureStandTurnLeft = 8,
		kGestureStandTurnRight = 9,
		kGestureBow = 10,
		kGestureRaiseHead = 11,
	};
	enum VoiceValue {
		kVoiceStandUp           = 1,
		kVoiceStandDown         = 2,
		kVoiceMoveForward       = 3,
		kVoiceMoveBackward      = 4,
		kVoiceMoveLeft          = 5,
		kVoiceMoveRight         = 6,
		kVoiceStop              = 7,
		kVoiceBow               = 8,
		kVoiceRaiseHead         = 9,
		kVoiceJump              = 10,
		kVoiceStandTurnLeft     = 11,
		kVoiceStandTurnRight    = 12,
		kVoiceTurnLeft          = 13,
		kVoiceTurnRight         = 14,
		kVoiceTurnBack          = 15,
		kVoiceTurnToTargetAngle = 16,
		kVoicePronkLong         = 17,
		kVoicePronkHigh         = 18,
		kVoicePrepareBackflip   = 19,
		kVoiceBackflip   				= 20,
		kVoiceDance   				= 21,
	};
};  // bascimethod
#endif  // BASIC_METHOD_H_
