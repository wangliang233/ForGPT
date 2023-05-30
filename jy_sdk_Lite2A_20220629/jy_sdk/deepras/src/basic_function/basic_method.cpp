
#include "basic_method.h"

static scope_output *scope_port;
static DataGlobalPreProcess global_buffer;
static DataGlobalPreProcess *global = &global_buffer;

Eigen::Matrix3d basicmethod::RolPitYawToRM(const DataGyro &gyro)
{
	Eigen::AngleAxisd yawAngle(gyro.yaw, (Eigen::Vector3d)Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd pitchAngle(gyro.pitch, (Eigen::Vector3d)Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd rollAngle(gyro.roll, (Eigen::Vector3d)Eigen::Vector3d::UnitX());
	Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
	return q.matrix();
}

Eigen::Matrix3d basicmethod::RPY_to_RM(const Vec3 &RPYs)                                    // RPY����ת����
{
	Matrix3d RM;
	double Rol = RPYs(0);
	double Pit = RPYs(1);
	double Yaw = RPYs(2);

	RM   <<cos(Yaw)*cos(Pit),  -sin(Yaw)*cos(Rol) + cos(Yaw)*sin(Pit)*sin(Rol),  sin(Yaw)*sin(Rol)+cos(Yaw)*sin(Pit)*cos(Rol),
			    sin(Yaw)*cos(Pit),  cos(Yaw)*cos(Rol) + sin(Yaw)*sin(Pit)*sin(Rol),  -cos(Yaw)*sin(Rol)+sin(Yaw)*sin(Pit)*cos(Rol),
			    -sin(Pit),               cos(Pit)*sin(Rol),                                             cos(Pit)*cos(Rol);

	return RM;
}

Eigen::Matrix3d basicmethod::RolPitYawToRM(const DataGyro &gyro, const int &i)
{
	Eigen::AngleAxisd yawAngle(gyro.yaw * i, (Eigen::Vector3d)Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd pitchAngle(gyro.pitch, (Eigen::Vector3d)Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd rollAngle(gyro.roll, (Eigen::Vector3d)Eigen::Vector3d::UnitX());
	Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
	return q.matrix();
}

Eigen::Matrix3d basicmethod::RolPitYawToRM(const Vec3 &angles)
{
	Eigen::AngleAxisd yawAngle(angles[2], (Eigen::Vector3d)Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd pitchAngle(angles[1], (Eigen::Vector3d)Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd rollAngle(angles[0], (Eigen::Vector3d)Eigen::Vector3d::UnitX());
	Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
	return q.matrix();
}

Eigen::Matrix3d basicmethod::RolPitYawToRM(const Vec3 &angles, const int &i)
{
	Eigen::AngleAxisd yawAngle(angles[2] * i, (Eigen::Vector3d)Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd pitchAngle(angles[1], (Eigen::Vector3d)Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd rollAngle(angles[0], (Eigen::Vector3d)Eigen::Vector3d::UnitX());
	Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
	return q.matrix();
}

Eigen::Matrix3d basicmethod::DerivRolPitYawToRM(const DataGyro &gyro)                            // RPY calculate the rotation matrix.
{
	double Rol = gyro.roll;
	double Pit = gyro.pitch;
	double Yaw = gyro.yaw;
	double dRol = gyro.rol_vel;
	double dPit = gyro.pit_vel;
	double dYaw = gyro.yaw_vel;

	Eigen::Matrix3d dR;
	dR << -cos(Yaw)* dPit *sin(Pit) - cos(Pit) *dYaw *sin(Yaw), -cos(Rol) * cos(Yaw) *dYaw + cos(Rol) * cos(Yaw) *dRol *sin(Pit) + cos(Pit) * cos(Yaw) * dPit *sin(Rol) + dRol *sin(Rol) *sin(Yaw) - dYaw* sin(Pit) *sin(Rol) *sin(Yaw), cos(Pit) * cos(Rol) * cos(Yaw) * dPit + cos(Yaw) *dYaw *sin(Rol) - cos(Yaw) *dRol* sin(Pit) *sin(Rol) + cos(Rol) *dRol *sin(Yaw) - cos(Rol) *dYaw *sin(Pit) *sin(Yaw),
		cos(Pit) * cos(Yaw) *dYaw - dPit* sin(Pit) *sin(Yaw), -cos(Yaw) * dRol *sin(Rol) + cos(Yaw) *dYaw* sin(Pit) * sin(Rol) - cos(Rol) *dYaw *sin(Yaw) + cos(Rol) *dRol *sin(Pit) *sin(Yaw) + cos(Pit) *dPit *sin(Rol) *sin(Yaw), -cos(Rol) *cos(Yaw) * dRol + cos(Rol) * cos(Yaw) *dYaw *sin(Pit) + cos(Pit) *cos(Rol) * dPit *sin(Yaw) + dYaw *sin(Rol) * sin(Yaw) - dRol *sin(Pit) * sin(Rol) *sin(Yaw),
		-cos(Pit) *dPit, cos(Pit) *cos(Rol)* dRol - dPit* sin(Pit)* sin(Rol), -cos(Rol) *dPit *sin(Pit) - cos(Pit)* dRol* sin(Rol);

	return dR;
}

Eigen::Matrix3d basicmethod::DerivRolPitYawToRM(const DataGyro &gyro, const int &i)                     // RPY calculate the rotation matrix.
{
	double Rol = gyro.roll;
	double Pit = gyro.pitch;
	double Yaw = gyro.yaw * i;
	double dRol = gyro.rol_vel;
	double dPit = gyro.pit_vel;
	double dYaw = gyro.yaw_vel * i;

	Eigen::Matrix3d dR;
	Eigen::Matrix3d body_mat, angular_vel_mat;
	body_mat = RolPitYawToRM(gyro,0);
	angular_vel_mat << 0, -dYaw, dPit,
										 dYaw, 0, -dRol,
										 -dPit, dRol, 0;

//	dR = angular_vel_mat * body_mat;
	dR = body_mat * angular_vel_mat;
	return dR;
}

int basicmethod::Sign(const double &i)
{
	if (i > 0) {
		return 1;
	} else if (i == 0) {
		return 0;
	} else {
		return -1;
	}
}

void basicmethod::GetAbsMin(const double &ref, double &own)
{
	if (abs(own) > abs(ref))
		own = Sign(own) * abs(ref);
}

void basicmethod::LimitRange(const double &min, const double &max, double &own)
{
	if (min > max) {
		cout << "[Error] Value Setting Error!! Please Check!!!" << endl;
	} else {
		if (own < min) {
			own = min;
		} else if (own > max) {
			own = max;
		}
	}
}

void basicmethod::GetAbsMax(const double &ref, double &own)
{
	if (abs(own) < abs(ref))
		own = Sign(own) * abs(ref);
}

void basicmethod::GetAbsMin(const Vec3 &ref, Vec3 &own)
{
	if (abs(own(0)) > abs(ref(0)))
		own(0) = Sign(own(0)) * abs(ref(0));
	if (abs(own(1)) > abs(ref(1)))
		own(1) = Sign(own(1)) * abs(ref(1));
	if (abs(own(2)) > abs(ref(2)))
		own(2) = Sign(own(2)) * abs(ref(2));
}

void basicmethod::GetMin(const double &ref, double &own)
{
	if (own > ref) own = ref;
}

void basicmethod::GetMax(const double &ref, double &own)
{
	if (own < ref) own = ref;
}

double basicmethod:: AbsLim(double bound, double a){
	double b;
	if (bound>0){
		b = bound;
	}else{
		b = -bound;
	}
	return max(min(a,b),-b);
}

void basicmethod::CubicSpline(double init_pos, double init_vel, double goal_pos, double goal_vel, double run_time, double cycle_time, double total_time, double &sub_goal_pos, double &sub_goal_pos_next)
{
	double a, b, c, d;
	d = init_pos;
	c = init_vel;
	a = (goal_vel * total_time - 2 * goal_pos + init_vel * total_time + 2 * init_pos) / pow(total_time, 3);
	b = (3 * goal_pos - goal_vel * total_time - 2 * init_vel*total_time - 3 * init_pos) / pow(total_time, 2);

	if (run_time > total_time)
		run_time = total_time;
	sub_goal_pos = a * pow(run_time, 3) + b * pow(run_time, 2) + c * run_time + d;

	if (run_time + cycle_time > total_time)
		run_time = total_time - cycle_time;
	sub_goal_pos_next = a * pow(run_time + cycle_time, 3) + b * pow(run_time + cycle_time, 2) + c * (run_time + cycle_time) + d;
}

void basicmethod::CubicSpline(double init_pos, double init_vel, double goal_pos, double goal_vel, double run_time, double cycle_time, double total_time, double &sub_goal_pos, double &sub_goal_pos_next, double &sub_goal_pos_next2)
{
	double a, b, c, d;
	d = init_pos;
	c = init_vel;
	a = (goal_vel * total_time - 2 * goal_pos + init_vel * total_time + 2 * init_pos) / pow(total_time, 3);
	b = (3 * goal_pos - goal_vel * total_time - 2 * init_vel*total_time - 3 * init_pos) / pow(total_time, 2);

	if (run_time > total_time)
		run_time = total_time;
	sub_goal_pos = a * pow(run_time, 3) + b * pow(run_time, 2) + c * run_time + d;

	if (run_time + cycle_time > total_time)
		run_time = total_time - cycle_time;
	sub_goal_pos_next = a * pow(run_time + cycle_time, 3) + b * pow(run_time + cycle_time, 2) + c * (run_time + cycle_time) + d;

	if (run_time + cycle_time * 2 > total_time)
		run_time = total_time - cycle_time * 2;
	sub_goal_pos_next2 = a * pow(run_time + cycle_time * 2, 3) + b * pow(run_time + cycle_time * 2, 2) + c * (run_time + cycle_time * 2) + d;
}

Vec3 basicmethod::VirtualForceCoordinateTransformFrom(const Vec3 &ori, const Vec3 &ang, const double &angle_hipy)
{
	Vec3 tmp;
	tmp << -angle_hipy, 0, 0;
	//return ori * RPY_to_RM(Ang, 0) * RPY_to_RM(tmp);
	return RolPitYawToRM(tmp).transpose() * RolPitYawToRM(ang, 0).transpose() * ori;  //Thesis equation (2.2).
}

Vec3 basicmethod::VirtualForceCoordinateTransformTo(const Vec3 &ori, const Vec3 &ang, const double &angle_hipy)
{
	Vec3 tmp;
	tmp << -angle_hipy, 0, 0;
	//return ori * RPY_to_RM(tmp).transpose() * RPY_to_RM(Ang, 0).transpose();
	return RolPitYawToRM(ang, 0) * RolPitYawToRM(tmp) * ori;
}

double basicmethod::CosineSpline(double init_pos, double goal_pos, double run_time, double total_time)
{
	double p_dh = 0.11;
	double a = -init_pos * p_dh;

	if (run_time < 0.5 * total_time) {
		return a * (0.5 - 0.5 * cos(2 * kPI * run_time / total_time)) + init_pos;
	} else if (run_time < total_time) {
		return (a + init_pos - goal_pos) * (0.5 - 0.5*cos(2 * kPI*run_time / total_time)) + goal_pos;
	} else {
		return goal_pos;
	}
}

double basicmethod::CosineSpline(double init_pos, double goal_pos, double run_time, double total_time, double p_dh)
{
	double a = -init_pos * p_dh;

	if (run_time < 0.3333 * total_time) {
		return a * (0.5 - 0.5 * cos(3 * kPI * run_time / total_time)) + init_pos;
	} else if (run_time < total_time) {
		return (a + init_pos - goal_pos) * (0.5 - 0.5 * cos(kPI * (1.5 * run_time / total_time + 0.5))) + goal_pos;
	} else {
		return goal_pos * (1 + 0.1 * (run_time - total_time) / total_time);
	}
}

void basicmethod::MotionConfigInitial(const string &robot_id, const string &robot_type, const MotionConfig &external_motion_config)
{
	string robot = robot_id.substr(0, 4);
	// ********** Default Configuration **********
	inner_motion_config.forward_velocity_offset.value = 0;
	inner_motion_config.side_velocity_offset.value = 0;	
	if (robot == "JY-M") {
		inner_motion_config.stand_height.value = 0.36;
	} else if(robot == "JY-P"){
		inner_motion_config.stand_height.value = 0.5;
	} else if(robot == "JY-S"){
		inner_motion_config.stand_height.value = 0.30;
	}  else {
		inner_motion_config.stand_height.value = 0.45;
	}
	// ********** Default Configuration **********

	// ********** Loading Configuration **********
	if (external_motion_config.forward_velocity_offset.valid) {
		inner_motion_config.forward_velocity_offset.value = external_motion_config.forward_velocity_offset.value;
	}
	if (external_motion_config.side_velocity_offset.valid) {
		inner_motion_config.side_velocity_offset.value = external_motion_config.side_velocity_offset.value;
	}
	if (external_motion_config.stand_height.valid) {
		inner_motion_config.stand_height.value = external_motion_config.stand_height.value;
	}
	memcpy(inner_motion_config.backflip_hip_kp,external_motion_config.backflip_hip_kp,sizeof(external_motion_config.backflip_hip_kp));	
	inner_motion_config.backflip_knee_kp = external_motion_config.backflip_knee_kp;
	inner_motion_config.sideflip_abad_kp = external_motion_config.sideflip_abad_kp;
	inner_motion_config.sideflip_hip_kp = external_motion_config.sideflip_hip_kp;
	inner_motion_config.sideflip_knee_kp = external_motion_config.sideflip_knee_kp;
	// ********** Loading Configuration **********

	// ********** Legalize Configuration **********
	GetAbsMin(0.2, inner_motion_config.forward_velocity_offset.value);
	GetAbsMin(0.05, inner_motion_config.side_velocity_offset.value);
	if (robot == "JY-M") {
		LimitRange(0.35, 0.4, inner_motion_config.stand_height.value);
	} else if (robot == "JY-S") {
		LimitRange(0.22, 0.33, inner_motion_config.stand_height.value);
	} else {
		LimitRange(0.4, 0.55, inner_motion_config.stand_height.value);
	}
	// ********** Legalize Configuration **********

	cout << "\n******** Motion Config Setting ********\n" << "Forward Vel Offset: " << inner_motion_config.forward_velocity_offset.value;
	if (!external_motion_config.forward_velocity_offset.valid) {
		cout << "(Default)";
	}
	cout << "\nSide Vel Offset: " << inner_motion_config.side_velocity_offset.value;
	if (!external_motion_config.side_velocity_offset.valid) {
		cout << "(Default)";
	}
	cout << "\nRobot Stand Height: " << inner_motion_config.stand_height.value;
	if (!external_motion_config.stand_height.valid) {
		cout << "(Default)";
	}

	cout <<"\n******** Motion Config Setting ********\n"<< endl;
}

void basicmethod::IsRobotLoseControl(RobotState &actual_robot_state_,RobotState &goal_robot_state_,const DataGyro &gyro_data_){
	if (abs(goal_robot_state_.roll_posture - gyro_data_.roll) > 35 * kDegree2Radian
			|| abs(goal_robot_state_.pitch_posture - gyro_data_.pitch) > 35 * kDegree2Radian
			|| abs(actual_robot_state_.side_vel) > 0.8 ) {  //switch to position control model.
		actual_robot_state_.is_robot_lose_control = true;
		cout<<"[Lose Control:]  goal_roll_posture:"<<goal_robot_state_.roll_posture<<"  goal_pitch_posture:"<<goal_robot_state_.pitch_posture<<"  actual_side_vel:"<<actual_robot_state_.side_vel<<endl;
	}
}

void basicmethod::DataPreProcessing(const string &full_robot_id, const int &control_state_record, RobotState &actual_robot_state, const DataGyro &gyro_data, const ControlCommands &control_commands, const double &abs_run_time,
									DataGyro *gyrp_data_process, InnerControlCommands *inner_control_commands, scope_output *scope, DataLegs *leg_output)
{
	GyroDataProcess(gyro_data, gyrp_data_process);
	memset(leg_output, 0, sizeof(leg_output));
	scope_port = scope;

	// ********** Working State Judge **********
	static int heat_print_cnt;

	bool is_motor_heat_warning = false;
	if (control_commands.motor_temperature != 0) {
		if (control_commands.motor_temperature > kWarningWorkingTemperature) {
			// heat_print_cnt ++;
			// if (heat_print_cnt == 100){
			 	cout << "[Warning:temp] Motor temp is high = " << control_commands.motor_temperature <<endl;
			// 	heat_print_cnt = 0;
			//}
			if (control_commands.motor_temperature > kMaxWorkingTemperature /*&& task_state.task_id == 0*/) {
				is_motor_heat_warning = true;
			}
		}
		else if (control_commands.motor_temperature < kWarningWorkingTemperature) is_motor_heat_warning = false;
	}

	bool is_drive_heat_warning = false;
	if (control_commands.driver_temperature != 0) {
		if (control_commands.driver_temperature > (kWarningWorkingTemperature-10)) {
			// heat_print_cnt ++;
			// if (heat_print_cnt == 100){
			 	cout << "[Warning:temp] Driver temp is high = " << control_commands.driver_temperature <<endl;
			// 	heat_print_cnt = 0;
			//}
			if (control_commands.driver_temperature > (kMaxWorkingTemperature-10) /*&& task_state.task_id == 0*/) {
				is_drive_heat_warning = true;
			}
		}
		else if (control_commands.driver_temperature < (kWarningWorkingTemperature-10)) is_drive_heat_warning = false;
	}
	bool is_heat_data_lose = control_commands.is_heat_data_lose;

	if (!control_commands.is_stop_button_on && ((control_commands.is_joystick_connect && !control_commands.is_wifi_lose) || !control_commands.is_app_lose)
		&& !is_heat_data_lose && !control_commands.is_drive_heat_warning && !is_motor_heat_warning
		&& !control_commands.is_gyro_data_lose  && global->is_gyro_connect && control_commands.is_driver_error == 0 && !control_commands.is_battary_low) {
		inner_control_commands->basic.is_robot_can_continue_work = true;
	} else {
		inner_control_commands->basic.is_robot_can_continue_work = false;
	}

	if (inner_control_commands->basic.is_robot_can_continue_work == false) {
		if (abs_run_time > global->robot_woking_state_judge_output_time_record + 8.0) {
			cout << "[Working State Judge] " << "soft stop button: " << !control_commands.is_stop_button_on << ", driver enable: " << (control_commands.is_driver_error == 0)
				<< ", wifi connect: " << !control_commands.is_wifi_lose << ", joystick connect: " << control_commands.is_joystick_connect <<"\ngyro connect: " << !control_commands.is_gyro_data_lose
				<< ", gyro nonzero: " << global->is_gyro_connect << ", heat data connect: " << !is_heat_data_lose <<", app connect: "<< !control_commands.is_app_lose
				<< ", motor heat normal: " << !is_motor_heat_warning << ", driver heat normal: " << !is_drive_heat_warning <<", battary voltage normal: " << !control_commands.is_battary_low << endl;
			global->robot_woking_state_judge_output_time_record = abs_run_time;
		}
	}

	if (control_commands.is_wifi_lose == true && robot_state_upload.robot_basic_state == 6 && control_commands.is_vision_mode == false 
		&& control_commands.vision_is_obstacle_distance_mode == false) {
		inner_control_commands->motion.user_forward_vel = 0;
		inner_control_commands->motion.user_side_vel = 0;
		inner_control_commands->motion.vision_forward_vel = 0;
		inner_control_commands->motion.vision_side_vel = 0;
		actual_robot_state.is_robot_lose_control = true;
	}

	if (control_commands.is_driver_error != 0 || global->soft_emergency_cnt != control_commands.soft_emergency_cnt) {
		actual_robot_state.is_robot_lose_control = true;
		inner_control_commands->basic.is_soft_emergency_pressed = true;
		if(global->soft_emergency_cnt != control_commands.soft_emergency_cnt){
			global->soft_emergency_cnt = control_commands.soft_emergency_cnt;
			cout<<"[Soft Emergency Pressed!!!]"<<endl;
		}
	}
	// ********** Working State Judge **********

	// ********** Navigation Commands **********
	inner_control_commands->motion.vision_forward_vel    = control_commands.vision_forward_vel;
	inner_control_commands->motion.vision_side_vel       = control_commands.vision_side_vel;
	inner_control_commands->motion.vision_turning_angle  = control_commands.vision_turning_angle;
	inner_control_commands->motion.vision_delay_time     = control_commands.vision_delay_time;
	inner_control_commands->motion.vision_location_x     = control_commands.vision_location_x;
	inner_control_commands->motion.vision_location_y     = control_commands.vision_location_y;
	inner_control_commands->motion.vision_location_theta = control_commands.vision_location_theta;
	inner_control_commands->basic.vision_location_x      = control_commands.vision_location_x;
	inner_control_commands->basic.vision_location_y      = control_commands.vision_location_y;
	inner_control_commands->basic.vision_location_theta  = control_commands.vision_location_theta;
	inner_control_commands->motion.is_vision_mode = control_commands.is_vision_mode;

	GetAbsMin(kSideVelocityMax, inner_control_commands->motion.vision_side_vel);
	GetAbsMin(kTurningVelocityMax, inner_control_commands->motion.vision_turning_vel);
	if (abs(inner_control_commands->motion.vision_turning_angle) < 2 * kDegree2Radian) {
		inner_control_commands->motion.vision_turning_angle = 0;
	}

	// ********** Navigation Commands **********

	// ********** User Control Commands **********
	if (control_commands.velocity_control_y == -1) {
		inner_control_commands->motion.user_side_vel = kRightVelocityLow;
	} else if (control_commands.velocity_control_y == 0) {
		inner_control_commands->motion.user_side_vel = 0;
	} else if (control_commands.velocity_control_y == 1) {
		inner_control_commands->motion.user_side_vel = kLeftVelocityLow;
	}

	inner_control_commands->basic.yaw = control_commands.right_axis_x * kYawMax;
	if(full_robot_id.substr(0,4) == "JY-L") 	inner_control_commands->basic.yaw = control_commands.right_axis_x * kYawMax * 0.6;
	inner_control_commands->basic.pitch = control_commands.right_axis_y*kPitchMax;
	if (abs(control_commands.left_axis_x/control_commands.left_axis_y) < 0.2 ){
		inner_control_commands->basic.z = control_commands.left_axis_y;
		inner_control_commands->basic.roll = 0;
	}else if (abs(control_commands.left_axis_y/control_commands.left_axis_x) < 0.2){
		inner_control_commands->basic.roll = control_commands.left_axis_x*kRollMax;
		inner_control_commands->basic.z = 0;

		if(control_commands.left_axis_x !=0 )inner_control_commands->motion.user_side_vel = -double(control_commands.left_axis_x) * 0.3;
	}else {
		inner_control_commands->basic.roll = control_commands.left_axis_x*kRollMax;
		inner_control_commands->basic.z = control_commands.left_axis_y;

		if(control_commands.left_axis_x !=0 )inner_control_commands->motion.user_side_vel = -double(control_commands.left_axis_x) * 0.3;
	}
  inner_control_commands->motion.user_side_vel = double(control_commands.LT_and_RT) * 0.2;
  inner_control_commands->motion.user_side_vel = control_commands.left_axis_x * 0.25;
	if (control_commands.left_axis_y == 0) {
		inner_control_commands->motion.user_forward_vel = 0;
	} else if (control_commands.left_axis_y < 0) {
		inner_control_commands->motion.user_forward_vel = control_commands.left_axis_y * kBackwardVelocityHigh;
		if(full_robot_id.substr(0, 4) == "JY-M" ) inner_control_commands->motion.user_forward_vel = control_commands.left_axis_y * kBackwardVelocityHighJYM;
    if(full_robot_id.substr(0, 4) == "JY-S" ) inner_control_commands->motion.user_forward_vel = 1.0*control_commands.left_axis_y * kBackwardVelocityHighJYS;
	} else if (control_commands.left_axis_y > 0) {
		inner_control_commands->motion.user_forward_vel = control_commands.left_axis_y * kForwardVelocityHigh;
		if(full_robot_id.substr(0, 4) == "JY-M" ) inner_control_commands->motion.user_forward_vel = control_commands.left_axis_y * kForwardVelocityHighJYM;
    if(full_robot_id.substr(0, 4) == "JY-S" ) inner_control_commands->motion.user_forward_vel = 1.4*control_commands.left_axis_y * kForwardVelocityHighJYS;
	}
	inner_control_commands->motion.user_turning_control = Sign(control_commands.right_axis_x) * 1;

	inner_control_commands->basic.stand_up_down_flag = control_commands.is_stand;
	inner_control_commands->basic.retry_charge = control_commands.retry_charge;
	inner_control_commands->basic.charge_over = control_commands.charge_over;
	inner_control_commands->basic.app_start_stop_flag = control_commands.app_start_stop_flag;
	inner_control_commands->basic.is_app_connect = !control_commands.is_app_lose;
	inner_control_commands->basic.raise_head_flag = control_commands.raise_head_flag;
	inner_control_commands->basic.switch_to_force_mode_flag = control_commands.switch_to_force_model;
	inner_control_commands->basic.turn_over_falg = control_commands.turn_over_flag;
	inner_control_commands->basic.start_stop_action_flag = control_commands.start_action_flag;
	inner_control_commands->basic.is_gesture_mode = control_commands.is_gesture_mode;
	inner_control_commands->basic.start_pronk_flag = control_commands.start_pronk_flag;
	inner_control_commands->basic.cmd_quick_roll = control_commands.cmd_quick_roll;
 	inner_control_commands->basic.zeroposition_flag = control_commands.start_pronk_flag;
 	inner_control_commands->basic.is_app_zeroposition = control_commands.is_zeroposition;
	inner_control_commands->basic.start_backflip_flag = control_commands.start_backflip_flag;	
	inner_control_commands->basic.is_backflip_mode = control_commands.is_backflip_mode;	
	inner_control_commands->basic.recover_direction_flag = control_commands.recover_direction_flag;
	inner_control_commands->basic.shot_cmd = control_commands.shot_cmd;
	inner_control_commands->basic.is_robot_continue_to_charge = global->is_robot_continue_to_charge;
	inner_control_commands->motion.change_swing_leg_flag = control_commands.change_swing_leg_flag;
	inner_control_commands->motion.change_body_height_flag = control_commands.switch_to_force_model;
	inner_control_commands->motion.adjust_peroid_flag = control_commands.turn_over_flag;
	inner_control_commands->motion.jump_type = control_commands.jump_type;

	// ********** User Control Commands **********

	// ********** Robot State Decision Making *********
	inner_control_commands->motion.is_robot_switch_state = false;
	string main_robot_id = full_robot_id.substr(0, 4);

	static bool is_robot_can_move;
	static double stop_time_record;

	inner_control_commands->basic.is_robot_can_move = false;

	inner_control_commands->target_robot_state = global->last_target_robot_state_record;

	global->last_target_robot_state_record = inner_control_commands->target_robot_state;
	global->last_control_state_record = control_state_record;

	inner_control_commands->motion.vision_side_vel    += inner_motion_config.side_velocity_offset.value;
	inner_control_commands->motion.user_side_vel      += inner_motion_config.side_velocity_offset.value;
	inner_control_commands->motion.vision_forward_vel += inner_motion_config.forward_velocity_offset.value;
	inner_control_commands->motion.user_forward_vel   += 0.00;

}

void basicmethod::OutputDataProcessing(const Vec3 &fl_pos, const Vec3 &fl_vel, const Vec3 &fl_torque, const Vec3 &fr_pos, const Vec3 &fr_vel, const Vec3 &fr_torque,
									   const Vec3 &hl_pos, const Vec3 &hl_vel, const Vec3 &hl_torque, const Vec3 &hr_pos, const Vec3 &hr_vel, const Vec3 &hr_torque, 
									   const OneLegControlParameter &control_parameters, DataLegs *leg_output)
{
	memcpy(leg_output->fl_pos.value, fl_pos.data(), sizeof(fl_pos));
	memcpy(leg_output->fr_pos.value, fr_pos.data(), sizeof(fr_pos));
	memcpy(leg_output->hl_pos.value, hl_pos.data(), sizeof(hl_pos));
	memcpy(leg_output->hr_pos.value, hr_pos.data(), sizeof(hr_pos));
	memcpy(leg_output->fl_vel.value, fl_vel.data(), sizeof(fl_vel));
	memcpy(leg_output->fr_vel.value, fr_vel.data(), sizeof(fr_vel));
	memcpy(leg_output->hl_vel.value, hl_vel.data(), sizeof(hl_vel));
	memcpy(leg_output->hr_vel.value, hr_vel.data(), sizeof(hr_vel));
	memcpy(leg_output->fl_torque.value, fl_torque.data(), sizeof(fl_torque));
	memcpy(leg_output->fr_torque.value, fr_torque.data(), sizeof(fr_torque));
	memcpy(leg_output->hl_torque.value, hl_torque.data(), sizeof(hl_torque));
	memcpy(leg_output->hr_torque.value, hr_torque.data(), sizeof(hr_torque));
	for (int i = 0; i < 3; ++i) {
		GetAbsMin(control_parameters.tor_lim[i], leg_output->fl_torque.value[i]);
		GetAbsMin(control_parameters.tor_lim[i], leg_output->fr_torque.value[i]);
		GetAbsMin(control_parameters.tor_lim[i], leg_output->hl_torque.value[i]);
		GetAbsMin(control_parameters.tor_lim[i], leg_output->hr_torque.value[i]);
	}
}

void basicmethod::AddToScope(const double data, const int num, const string var_name)
{
	if (num >= kMaxScopeNum) {
		cout << "Scope overflow!!!" << endl;
	} else {
		(*scope_port)[num] = data;
		if (global->scope_object[num][0] == "") {
			global->scope_object[num][0] = var_name;
		} else if (global->scope_object[num][0] != var_name) {
			global->scope_object[num][1] = "...";
		}
	}
}

void basicmethod::ScopeNamesOutput()
{
	cout << "ScopeNamesMap (Size:" << kMaxScopeNum << ")" << "\n";
	for (int i = 0; i < kMaxScopeNum; i++) {
		if (global->scope_object[i][0] != "") {
			cout << "No." << i << ": " << global->scope_object[i][0] << global->scope_object[i][1] << "\n";
		}
	}
	cout << endl;
}

void basicmethod::RobotStateOutput(const RobotState &goal_robot_state, const RobotState &actual_robot_state)
{
}

void basicmethod::GyroDataProcess(const DataGyro &gyro_data_raw, DataGyro *gyro_data_process)
{
	gyro_data_process->roll = gyro_data_raw.roll * kDegree2Radian;
	gyro_data_process->pitch = gyro_data_raw.pitch * kDegree2Radian;
	gyro_data_process->yaw = gyro_data_raw.yaw * kDegree2Radian;
	gyro_data_process->rol_vel = gyro_data_raw.rol_vel;
	gyro_data_process->pit_vel = gyro_data_raw.pit_vel;
	gyro_data_process->yaw_vel = gyro_data_raw.yaw_vel;
	gyro_data_process->acc_x = gyro_data_raw.acc_x;
	gyro_data_process->acc_y = gyro_data_raw.acc_y;
	gyro_data_process->acc_z = gyro_data_raw.acc_z;

	if (global->yaw_angle_first_record) {
		if (abs(global->last_cycle_yaw_angle - gyro_data_process->yaw) > 5.0 / 3 * kPI) {
			if (global->last_cycle_yaw_angle > gyro_data_process->yaw) {
				global->yaw_angle_cycle++;
			} else {
				global->yaw_angle_cycle--;
			}
		}
		global->last_cycle_yaw_angle = gyro_data_process->yaw;
		gyro_data_process->yaw = gyro_data_process->yaw + 2 * kPI * global->yaw_angle_cycle;
	}
	if (!global->yaw_angle_first_record) {
		global->last_cycle_yaw_angle = gyro_data_process->yaw;
		global->yaw_angle_first_record = true;
	}

	if (gyro_data_raw.roll != 0 || gyro_data_raw.pitch != 0 || gyro_data_raw.yaw != 0) {
		global->is_gyro_connect = true;
	} else {
		global->is_gyro_connect = false;
	}
}

