
#include "one_leg_control.h"
#include "parameters/parameters_share.h"
#include "controller/external_type.h"
DataGlobalOneLeg LegControl::global_;

LegControl::LegControl(int i)
{
  leg_id = i;
  kk0 = 0.25;
  kk1 = 1.0;
  kk2 = 0.6; 
	Initial();
}

LegControl::~LegControl()
{

}

void LegControl::ConstParameterInitial(const string &robot_id, const string &leg)
{
	robot_id_ = robot_id.substr(0, 4);
	if (robot_id_ == "JY-S") {
		if (leg == "FL") {
			link_ = kHardwareParameterJYS.fl;
		} else if (leg == "FR") {
			link_ = kHardwareParameterJYS.fr;
		} else if (leg == "HL") {
			link_ = kHardwareParameterJYS.hl;
		} else if (leg == "HR") {
			link_ = kHardwareParameterJYS.hr;
		}
		control_parameter_ = kDefaultControlParameterJYS.leg;
	}else {
		cout << "Const parameters initial failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		cout << "Please shut down the program!!!" << endl;
		while (true) {

		}
	}
}

void LegControl::DataInputProcess(const OneTypeLegData &leg_pos_input, const OneTypeLegData &leg_vel_input, const OneTypeLegData &leg_torque_input,
								  const DataGyro &gyro_data_process, const double &cycle_time)
{
	angle_ << leg_pos_input.value[0], leg_pos_input.value[1], leg_pos_input.value[2];
	velocity_ << leg_vel_input.value[0], leg_vel_input.value[1], leg_vel_input.value[2];
	real_torque_ << leg_torque_input.value[0], leg_torque_input.value[1], leg_torque_input.value[2];
	goal_angle_ << 0, 0, 0;
	goal_ang_vel_ << 0, 0, 0;
	torque_output_ << 0, 0, 0;
	gyro_data_ = gyro_data_process;
	cycle_time_ = cycle_time;
	hip_states_global_ = GetLegPositionGlobal();
	hip_states_local_ = CalculateHipStatesLocal();
// memset(&download_parameters, 0, sizeof(download_parameters));
}

void LegControl::Initial()
{
	goal_angle_ << 0, 0, 0;
	goal_ang_vel_ << 0, 0, 0;
	torque_output_ << 0, 0, 0;
	//toe_angles_ = CalculateToeAngles();
	//jacobian_ = CalculateJacobian();
	//hip_states_global_ = CalculateHipStatesGlobal();
}

Vec3 LegControl::CalculateAngleGlobal(Vec3 goal_xyz)
{
	Vec3 goal_local_xyz;

	goal_local_xyz = RolPitYawToRM(gyro_data_, 0).inverse() * goal_xyz;
	return InverseKinematic(goal_local_xyz);
}

Vec3 LegControl::CalculateAngleConsiderPitch(Vec3 goal_xyz)
{
	Vec3 goal_local_xyz, gyro_ang;

	gyro_ang << 0, gyro_data_.pitch, 0;
	goal_local_xyz = RolPitYawToRM(gyro_ang).transpose() * goal_xyz;
	return InverseKinematic(goal_local_xyz);
}

Vec3 LegControl::CalculateAngleLocal(Vec3 goal_xyz)
{
	return InverseKinematic(goal_xyz);
}

Vec3 LegControl::InverseKinematic(Vec3 goal_xyz)
{
	Vec3 angles;
	double x, y, z;
	double l0, l1, l2;
	double s1, s2, s3;

	l0 = link_.link_length[0]; l1 = link_.link_length[1]; l2 = link_.link_length[2];
	x = goal_xyz[0]; y = goal_xyz[1]; z = goal_xyz[2];

	s1 = atan(y / z) - asin(l0 / sqrt(y*y + z*z));

	s3 = acos((x*x + y*y + z*z - l0*l0 - l1*l1 - l2*l2) / (2 * l1*l2));

	s2 = asin(x / sqrt((l1 + l2*cos(s3))*(l1 + l2*cos(s3)) + l2*sin(s3)*l2*sin(s3))) - atan(l2*sin(s3) / (l1 + l2*cos(s3)));

	angles << s1, s2, s3;
	return angles;
}

Vec3 LegControl::InverseKinematicGlobal(Vec3 goal_xyz)
{
	Vec3 goal_xyz_global = RolPitYawToRM(gyro_data_, 0).transpose() * goal_xyz;
	return LegControl::InverseKinematic(goal_xyz_global);
}

void LegControl::SwingToPosition(int n)
{
	Vec3 goal_angle_next;

	if (n == 0) {
		goal_angle_ = CalculateAngleLocal(goal_pos_);
		goal_angle_next = CalculateAngleLocal(goal_pos_next_);
	} else {
		goal_angle_ = CalculateAngleGlobal(goal_pos_);
		goal_angle_next = CalculateAngleGlobal(goal_pos_next_);
	}
	goal_ang_vel_ = (goal_angle_next - goal_angle_) / cycle_time_;

	torque_output_[0] = control_parameter_.hipx_kp * (goal_angle_[0] - angle_[0]) + control_parameter_.hipx_kd * (goal_ang_vel_[0] - velocity_[0]);
	torque_output_[1] = control_parameter_.hipy_kp * (goal_angle_[1] - angle_[1]) + control_parameter_.hipy_kd * (goal_ang_vel_[1] - velocity_[1]);
	torque_output_[2] = control_parameter_.knee_kp * (goal_angle_[2] - angle_[2]) + control_parameter_.knee_kd * (goal_ang_vel_[2] - velocity_[2]);

}

void LegControl::SwingToPosition(double *goal_pos, string side)
{
	Vec3 goal_position(goal_pos);

	SwingToPosition(goal_position, goal_position, goal_position, 0, side);
}

void LegControl::SwingToPositionAnglePlan(Vec3 initial_angle, Vec3 goal_xyz_angle, double total_time, double run_time, string side, int i)
{
	Vec3 goal_angle_next;
	Vec3 goal_angle_next2;
	Vec3 final_angle;

	if (i == 0) {
		final_angle = CalculateAngleLocal(goal_xyz_angle);
	} else if (i == 1) {
		final_angle = goal_xyz_angle;
	}
	for (int j = 0; j<3; j++) {
		CubicSpline(initial_angle[j], 0, (double)final_angle[j], 0,
			run_time, cycle_time_, total_time, goal_angle_[j], goal_angle_next[j], goal_angle_next2[j]);
	}
	if (i == 2) {
		goal_angle_ = goal_xyz_angle;
		goal_angle_next = goal_xyz_angle;
		goal_angle_next2 = goal_xyz_angle;
	}

	goal_ang_vel_ = (goal_angle_next - goal_angle_) / cycle_time_;
	goal_ang_vel_next_ = (goal_angle_next2 - goal_angle_next) / cycle_time_;
	goal_ang_acc_ = (goal_ang_vel_next_ - goal_ang_vel_) / cycle_time_;

	double k1, k2, k3;
	k1 = 0;
	k2 = 0.0;
	k3 = 1;
	double M_hipx = 0;//0.15;  //0.123
	Vec3 coulombs; Vec3 low_vel_compensation;coulombs << 0, 0, 0;low_vel_compensation << 0, 0, 0;
	//fr_knee=1;fr_hipy=1.3;fr_hipx=1;
	//hr_knee=1.8;hr_hipy=2;hr_hipx=1;
	//fl_knee=1;fl_hipy=1.5;fl_hipx=1;
	//hl_knee=0.75;fl_hipy=4;fl_hipx=3;
	if (side == "FR") {
		global_.fr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(control_parameter_.hipx_kp) * (double(double(goal_angle_[0]) - double(angle_[0]))) + double(control_parameter_.hipx_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
			+ kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] + control_parameter_.gravity_coef3 * kDegree2Radian));//+0.00756*9.5*kGravity*cos(angle_[0]);

		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.3*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];

	}
	else if (side == "FL") {
		global_.fl_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(double(control_parameter_.hipx_kp) * (double(double(goal_angle_[0]) - double(angle_[0]))) + double(control_parameter_.hipx_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
			 + kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] - control_parameter_.gravity_coef3 * kDegree2Radian)));//-0.00756*9.5*kGravity*cos(angle_[0]);

		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.5*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];
	}
	else if (side == "HR") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(control_parameter_.hipx_kp) * (double(double(goal_angle_[0]) - double(angle_[0]))) + double(control_parameter_.hipx_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
			 + kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] + control_parameter_.gravity_coef3 * kDegree2Radian));//+0.00756*9.5*kGravity*cos(angle_[0]);

		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 2 * coulombs[1];
		low_vel_compensation[2] = 2 * coulombs[2];

	}
	else if (side == "HL") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(control_parameter_.hipx_kp) * (double(double(goal_angle_[0]) - double(angle_[0]))) + double(control_parameter_.hipx_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
			 + kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] - control_parameter_.gravity_coef3 * kDegree2Radian));//-0.00756*9.5*kGravity*cos(angle_[0]);

		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 3 * coulombs[0];
		low_vel_compensation[1] = 4 * coulombs[1];
		low_vel_compensation[2] = 0.75*coulombs[2];
	}

	double T1, T2, T0;
	DynamicsCompensation(goal_angle_, goal_angle_next, goal_angle_next2, T1, T2, T0);
	if (T1 >= 50) {
		T1 = 50;
	}
	if (T1 <= -50) {
		T1 = -50;
	}
	if (T2 >= 50) {
		T2 = 50;
	}
	if (T2 <= -50) {
		T2 = -50;
	}
	if (T0 >= 50) {
		T0 = 50;
	}
	if (T0 <= -50) {
		T0 = -50;
	}

  static double temp = angle_[2];
	if(side == "FR"){
		AddToScope(angle_[2], 31, "");
		AddToScope(goal_angle_[2], 32, "");
    AddToScope((angle_[2] - temp)/cycle_time_, 33, "");
    AddToScope(goal_ang_vel_[2], 34, "");
    AddToScope(velocity_[2], 35, "");
	}
  temp = angle_[2];
	//	T0=0;T1=0;T2=0;
	//T0,T1,T2 is dynamic compensation,effect in high speed situation; low_vel_compensation is coulombs friction,effect in low speed situation

	torque_output_[1] = control_parameter_.hipy_kp * (goal_angle_[1] - angle_[1]) + control_parameter_.hipy_kd * (goal_ang_vel_[1] - velocity_[1])+ kk1 * T1 +0* control_parameter_.low_vel * low_vel_compensation[1] ;//+ T1 + control_parameter_.low_vel * low_vel_compensation[1];
	torque_output_[2] = control_parameter_.knee_kp * (goal_angle_[2] - angle_[2]) + control_parameter_.knee_kd * (goal_ang_vel_[2] - velocity_[2])+ kk2 * T2 +0* control_parameter_.low_vel * low_vel_compensation[2];//+ T2 + control_parameter_.low_vel * low_vel_compensation[2];
	torque_output_[0] = double(torque_output_[0] + T0 + 0*control_parameter_.low_vel * low_vel_compensation[0]);
 // test 
 	for(int i = 0; i < 4; i++){
		download_parameters.kp[3*i]=control_parameter_.hipx_kp;
		download_parameters.kp[3*i+1]=control_parameter_.hipy_kp;
		download_parameters.kp[3*i+2]=control_parameter_.knee_kp;
		download_parameters.kd[3*i]=control_parameter_.hipx_kd;
		download_parameters.kd[3*i+1]=control_parameter_.hipy_kd;
		download_parameters.kd[3*i+2]=control_parameter_.knee_kd;
	}
   if(side == "FL") {
   download_parameters.t_ff[0] =  -kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[0] = goal_angle_[0];
   download_parameters.goal_angle_vel[0] = goal_ang_vel_[0];
   download_parameters.t_ff[1] = kk1*T1;
   download_parameters.goal_angle[1] = goal_angle_[1];
   download_parameters.goal_angle_vel[1] = goal_ang_vel_[1];
   download_parameters.t_ff[2] = kk2*T2;
   download_parameters.goal_angle[2] = goal_angle_[2];
   download_parameters.goal_angle_vel[2] = goal_ang_vel_[2];
   }
   if(side == "FR") {
   download_parameters.t_ff[3] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[3] = goal_angle_[0];
   download_parameters.goal_angle_vel[3] = goal_ang_vel_[0];
   download_parameters.t_ff[4] = kk1 * T1;
   download_parameters.goal_angle[4] = goal_angle_[1];
   download_parameters.goal_angle_vel[4] = goal_ang_vel_[1];
   download_parameters.t_ff[5] = kk2 * T2;
   download_parameters.goal_angle[5] = goal_angle_[2];
   download_parameters.goal_angle_vel[5] = goal_ang_vel_[2];
   }
   if(side == "HL") {
   download_parameters.t_ff[6] = -kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[6] = goal_angle_[0];
   download_parameters.goal_angle_vel[6] = goal_ang_vel_[0];
   download_parameters.t_ff[7] =kk1 *  T1;
   download_parameters.goal_angle[7] = goal_angle_[1];
   download_parameters.goal_angle_vel[7] = goal_ang_vel_[1];
   download_parameters.t_ff[8] = kk2 * T2;
   download_parameters.goal_angle[8]= goal_angle_[2];
   download_parameters.goal_angle_vel[8] = goal_ang_vel_[2];
   }
   if(side == "HR") {
   download_parameters.t_ff[9] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[9] = goal_angle_[0];
   download_parameters.goal_angle_vel[9] = goal_ang_vel_[0];
   download_parameters.t_ff[10] = kk1 * T1;
   download_parameters.goal_angle[10] = goal_angle_[1];
   download_parameters.goal_angle_vel[10] = goal_ang_vel_[1];
   download_parameters.t_ff[11] =kk2 *  T2;
   download_parameters.goal_angle[11] = goal_angle_[2];
   download_parameters.goal_angle_vel[11] = goal_ang_vel_[2];
   }
   
 //
	knee_vel_offset_ = goal_ang_vel_[2] - velocity_[2];
	knee_angle_offset_ = abs(goal_angle_[2] - angle_[2]);
	hipy_vel_offset_ = abs(goal_ang_vel_[1] - velocity_[1]);
	hipy_angle_offset_ = abs(goal_angle_[1] - angle_[1]);
}

void LegControl::SwingToPosition(Vec3 goal_pos, Vec3 goal_pos_next, Vec3 goal_pos_next2, int n, string side)
{
	Vec3 goal_angle_next;
	Vec3 goal_angle_next2;

	switch (n) {
	case 0: {
		goal_angle_ =CalculateAngleLocal(goal_pos); //goal_pos;//CalculateAngleLocal(goal_pos);
		goal_angle_next = CalculateAngleLocal(goal_pos_next);//goal_pos_next;//CalculateAngleLocal(goal_pos_next);
		goal_angle_next2 =CalculateAngleLocal(goal_pos_next2);//goal_pos_next2;//CalculateAngleLocal(goal_pos_next2);
		break;
	}

	case 2: {
		goal_angle_ = CalculateAngleConsiderPitch(goal_pos);
		goal_angle_next = CalculateAngleConsiderPitch(goal_pos_next);
		goal_angle_next2 = CalculateAngleConsiderPitch(goal_pos_next2);
		break;
	}

	case 1: {
		goal_angle_ = CalculateAngleGlobal(goal_pos);
		goal_angle_next = CalculateAngleGlobal(goal_pos_next);
		goal_angle_next2 = CalculateAngleGlobal(goal_pos_next2);
		break;
	}

	default: {
		goal_angle_ = CalculateAngleGlobal(goal_pos);
		goal_angle_next = CalculateAngleGlobal(goal_pos_next);
		goal_angle_next2 = CalculateAngleGlobal(goal_pos_next2);
		break;
	}
	}
	goal_ang_vel_ = (goal_angle_next - goal_angle_) / cycle_time_;
	goal_ang_vel_next_ = (goal_angle_next2 - goal_angle_next) / cycle_time_;
	goal_ang_acc_ = (goal_ang_vel_next_ - goal_ang_vel_) / cycle_time_;
 if(side == "FL")
 {
 AddToScope(goal_ang_acc_[0], 53, "");
 }

	double k1, k2, k3;
	k1 = 0.0;
	k2 = 0.0;
	k3 = 1;
	double M_hipx = 0.0;//0.15;  //0.123
	Vec3 coulombs; Vec3 low_vel_compensation;coulombs << 0, 0, 0;low_vel_compensation << 0, 0, 0;
	//fr_knee=1;fr_hipy=1.3;fr_hipx=1;
	//hr_knee=1.8;hr_hipy=2;hr_hipx=1;
	//fl_knee=1;fl_hipy=1.5;fl_hipx=1;
	//hl_knee=0.75;fl_hipy=4;fl_hipx=3;
  
	if (side == "FR") {
		global_.fr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(control_parameter_.hipx_kp) * (double(double(goal_angle_[0]) - double(angle_[0]))) + double(control_parameter_.hipx_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
			+ kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0]));//+0.00756*9.5*kGravity*cos(angle_[0]);

		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.3*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];

	}
	else if (side == "FL") {
		global_.fl_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(double(control_parameter_.hipx_kp) * (double(double(goal_angle_[0]) - double(angle_[0]))) + double(control_parameter_.hipx_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
			 - kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0] )));
    AddToScope(torque_output_[0], 50, "");
		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.5*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];
	}
	else if (side == "HR") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(control_parameter_.hipx_kp) * (double(double(goal_angle_[0]) - double(angle_[0]))) + double(control_parameter_.hipx_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
			 + kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0]));//+0.00756*9.5*kGravity*cos(angle_[0]);

		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 2 * coulombs[1];
		low_vel_compensation[2] = 2 * coulombs[2];

	}
	else if (side == "HL") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(control_parameter_.hipx_kp) * (double(double(goal_angle_[0]) - double(angle_[0]))) + double(control_parameter_.hipx_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
			 - kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0]));//-0.00756*9.5*kGravity*cos(angle_[0]);

		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 3 * coulombs[0];
		low_vel_compensation[1] = 4 * coulombs[1];
		low_vel_compensation[2] = 0.75*coulombs[2];
	}
  
	double T1, T2, T0;
	DynamicsCompensation(goal_angle_, goal_angle_next, goal_angle_next2, T1, T2, T0);
	if (T1 >= 50) {
		T1 = 50;
	}
	if (T1 <= -50) {
		T1 = -50;
	}
	if (T2 >= 50) {
		T2 = 50;
	}
	if (T2 <= -50) {
		T2 = -50;
	}
	if (T0 >= 50) {
		T0 = 50;
	}
	if (T0 <= -50) {
		T0 = -50;
	}


  static double temp = angle_[2];
	if(side == "HR"){
// AddToScope(torque_output_[0], 50, "");
	//	AddToScope(angle_[2], 31, "");
	//	AddToScope(goal_angle_[2], 32, "");
  //  //AddToScope((angle_[2] - temp)/cycle_time_, 33, "");
   // AddToScope(goal_ang_vel_[2], 34, "");
   // AddToScope(velocity_[2], 35, "");
  //  AddToScope(control_parameter_.knee_kp * (goal_angle_[2] - angle_[2]) + control_parameter_.knee_kd * (goal_ang_vel_[2] - velocity_[2]), 36, "");
  //  AddToScope(control_parameter_.knee_kp * (goal_angle_[2] - angle_[2]), 37, "");
 //   AddToScope(control_parameter_.knee_kd * (goal_ang_vel_[2] - velocity_[2]), 38, "");
	}
  temp = angle_[2];

	//T0,T1,T2 is dynamic compensation,effect in high speed situation; low_vel_compensation is coulombs friction,effect in low speed situation
	torque_output_[1] = control_parameter_.hipy_kp * (goal_angle_[1] - angle_[1]) + control_parameter_.hipy_kd * (goal_ang_vel_[1] - velocity_[1]) + kk1 * T1 + 0*control_parameter_.low_vel * low_vel_compensation[1];
	torque_output_[2] = control_parameter_.knee_kp * (goal_angle_[2] - angle_[2]) +control_parameter_.knee_kd * (goal_ang_vel_[2] - velocity_[2]) + kk2 * T2 + 0*control_parameter_.low_vel * low_vel_compensation[2];
	torque_output_[0] = double(torque_output_[0] +T0 +0 * control_parameter_.low_vel * low_vel_compensation[0]);
	if(side == "FL"){	
	    //torque_output_[1] = 60 * (goal_angle_[1] - angle_[1]) + 1.4 * (goal_ang_vel_[1] - velocity_[1]) + T1 + 0*control_parameter_.low_vel * low_vel_compensation[1];
     //AddToScope(control_parameter_.hipy_kp * (goal_angle_[1] - angle_[1]) + control_parameter_.hipy_kd * (goal_ang_vel_[1] - velocity_[1]), 50, "");
		 AddToScope(T0, 51, "");
     AddToScope(torque_output_[0], 52, "");
   }
   // test 
 	for(int i = 0; i < 4; i++){
		download_parameters.kp[3*i]=control_parameter_.hipx_kp;
		download_parameters.kp[3*i+1]=control_parameter_.hipy_kp;
		download_parameters.kp[3*i+2]=control_parameter_.knee_kp;
		download_parameters.kd[3*i]=control_parameter_.hipx_kd;
		download_parameters.kd[3*i+1]=control_parameter_.hipy_kd;
		download_parameters.kd[3*i+2]=control_parameter_.knee_kd;
	}   

   if(side == "FL") {
   download_parameters.t_ff[0] =  -kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[0] = goal_angle_[0];
   download_parameters.goal_angle_vel[0] = goal_ang_vel_[0];
   download_parameters.t_ff[1] = kk1*T1;
   download_parameters.goal_angle[1] = goal_angle_[1];
   download_parameters.goal_angle_vel[1] = goal_ang_vel_[1];
   download_parameters.t_ff[2] = kk2*T2;
   download_parameters.goal_angle[2] = goal_angle_[2];
   download_parameters.goal_angle_vel[2] = goal_ang_vel_[2];
   }
   if(side == "FR") {
   download_parameters.t_ff[3] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[3] = goal_angle_[0];
   download_parameters.goal_angle_vel[3] = goal_ang_vel_[0];
   download_parameters.t_ff[4] = kk1 * T1;
   download_parameters.goal_angle[4] = goal_angle_[1];
   download_parameters.goal_angle_vel[4] = goal_ang_vel_[1];
   download_parameters.t_ff[5] = kk2 * T2;
   download_parameters.goal_angle[5] = goal_angle_[2];
   download_parameters.goal_angle_vel[5] = goal_ang_vel_[2];
   }
   if(side == "HL") {
   download_parameters.t_ff[6] = -kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[6] = goal_angle_[0];
   download_parameters.goal_angle_vel[6] = goal_ang_vel_[0];
   download_parameters.t_ff[7] =kk1 *  T1;
   download_parameters.goal_angle[7] = goal_angle_[1];
   download_parameters.goal_angle_vel[7] = goal_ang_vel_[1];
   download_parameters.t_ff[8] = kk2 * T2;
   download_parameters.goal_angle[8]= goal_angle_[2];
   download_parameters.goal_angle_vel[8] = goal_ang_vel_[2];
   }
   if(side == "HR") {
   download_parameters.t_ff[9] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[9] = goal_angle_[0];
   download_parameters.goal_angle_vel[9] = goal_ang_vel_[0];
   download_parameters.t_ff[10] = kk1 * T1;
   download_parameters.goal_angle[10] = goal_angle_[1];
   download_parameters.goal_angle_vel[10] = goal_ang_vel_[1];
   download_parameters.t_ff[11] =kk2 *  T2;
   download_parameters.goal_angle[11] = goal_angle_[2];
   download_parameters.goal_angle_vel[11] = goal_ang_vel_[2];
   }
   
 //
	knee_vel_offset_ = goal_ang_vel_next_[2] - velocity_[2];
	knee_angle_offset_ = abs(goal_angle_[2] - angle_[2]);
	hipy_vel_offset_ = abs(goal_ang_vel_[1] - velocity_[1]);
	hipy_angle_offset_ = abs(goal_angle_[1] - angle_[1]);
}

void LegControl::SwingToPositionStandUpAndDown(Vec3 goal_pos, Vec3 goal_pos_next, Vec3 goal_pos_next2, string side){
	Vec3 goal_angle_next;
	Vec3 goal_angle_next2;
	goal_angle_ = CalculateAngleLocal(goal_pos);
	goal_angle_next = CalculateAngleLocal(goal_pos_next);
	goal_angle_next2 = CalculateAngleLocal(goal_pos_next2);

	goal_ang_vel_ = (goal_angle_next - goal_angle_) / cycle_time_;
	goal_ang_vel_next_ = (goal_angle_next2 - goal_angle_next) / cycle_time_;
	goal_ang_acc_ = (goal_ang_vel_next_ - goal_ang_vel_) / cycle_time_;

	double k1, k2, k3;
	k1 = 0.0;
	k2 = 0.0;
	k3 = 1.0;
	double M_hipx = 0.0;//0.15;  //0.123
	Vec3 coulombs; Vec3 low_vel_compensation;coulombs << 0, 0, 0;low_vel_compensation << 0, 0, 0;

	double hip_x_kp = control_parameter_.hipx_kp;
	double hip_y_kp = control_parameter_.hipy_kp;
	double knee_kp  = control_parameter_.knee_kp;
	double hip_x_kd = control_parameter_.hipx_kd;
	double hip_y_kd = control_parameter_.hipy_kd;
	double knee_kd  = control_parameter_.knee_kd;

	if (robot_id_ == "JY-M"){
		hip_x_kp = 200.0;
		hip_x_kd = 1.8;
		hip_y_kp = 200.0;
		hip_y_kd = 1.8;
		knee_kp  = 300.0;
		knee_kd  = 3.0;
	}

	if (side == "FR") {
		global_.fr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(hip_x_kp) * (double(double(goal_angle_[0]) - double(angle_[0])))
											+ double(hip_x_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
			                +kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] + control_parameter_.gravity_coef3 * kDegree2Radian));//+0.00756*9.5*kGravity*cos(angle_[0]);
		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.3*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];

	}
	else if (side == "FL") {
		global_.fl_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(double(hip_x_kp) * (double(double(goal_angle_[0]) - double(angle_[0])))
											+ double(hip_x_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
											+kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] - control_parameter_.gravity_coef3 * kDegree2Radian)));//-0.00756*9.5*kGravity*cos(angle_[0]);
		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.5*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];
	}
	else if (side == "HR") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(hip_x_kp) * (double(double(goal_angle_[0]) - double(angle_[0])))
											+ double(hip_x_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
											+kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] + control_parameter_.gravity_coef3 * kDegree2Radian));//+0.00756*9.5*kGravity*cos(angle_[0]);
		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 2 * coulombs[1];
		low_vel_compensation[2] = 2 * coulombs[2];

	}
	else if (side == "HL") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = double(hip_x_kp) * (double(double(goal_angle_[0]) - double(angle_[0])))
											+ double(hip_x_kd) * (double(goal_ang_vel_[0] - velocity_[0]))
											+kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] - control_parameter_.gravity_coef3 * kDegree2Radian));//-0.00756*9.5*kGravity*cos(angle_[0]);
		if (robot_id_ == "JY-C") {
			CoulombsCoefficient(coulombs, goal_ang_vel_);
		}
		low_vel_compensation[0] = 3 * coulombs[0];
		low_vel_compensation[1] = 4 * coulombs[1];
		low_vel_compensation[2] = 0.75*coulombs[2];
	}
	double T1, T2, T0;
	DynamicsCompensation(goal_angle_, goal_angle_next, goal_angle_next2, T1, T2, T0);
	if (T1 >= 50) {
		T1 = 50;
	}
	if (T1 <= -50) {
		T1 = -50;
	}
	if (T2 >= 50) {
		T2 = 50;
	}
	if (T2 <= -50) {
		T2 = -50;
	}
	if (T0 >= 50) {
		T0 = 50;
	}
	if (T0 <= -50) {
		T0 = -50;
	}

	torque_output_[1] = hip_y_kp * (goal_angle_[1] - angle_[1]) + hip_y_kd * (goal_ang_vel_[1] - velocity_[1]) + kk1 * T1 +0* control_parameter_.low_vel * low_vel_compensation[1];
	torque_output_[2] = knee_kp * (goal_angle_[2] - angle_[2]) + knee_kd * (goal_ang_vel_[2] - velocity_[2]) + kk2 * T2 + 0*control_parameter_.low_vel * low_vel_compensation[2];
	torque_output_[0] = double(torque_output_[0] + T0 + 0*control_parameter_.low_vel * low_vel_compensation[0]);
  // test 
 	for(int i = 0; i < 4; i++){
		download_parameters.kp[3*i]=control_parameter_.hipx_kp;
		download_parameters.kp[3*i+1]=control_parameter_.hipy_kp;
		download_parameters.kp[3*i+2]=control_parameter_.knee_kp;
		download_parameters.kd[3*i]=control_parameter_.hipx_kd;
		download_parameters.kd[3*i+1]=control_parameter_.hipy_kd;
		download_parameters.kd[3*i+2]=control_parameter_.knee_kd;
	}   

   if(side == "FL") {
   download_parameters.t_ff[0] =  -kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[0] = goal_angle_[0];
   download_parameters.goal_angle_vel[0] = goal_ang_vel_[0];
   download_parameters.t_ff[1] = kk1*T1;
   download_parameters.goal_angle[1] = goal_angle_[1];
   download_parameters.goal_angle_vel[1] = goal_ang_vel_[1];
   download_parameters.t_ff[2] = kk2*T2;
   download_parameters.goal_angle[2] = goal_angle_[2];
   download_parameters.goal_angle_vel[2] = goal_ang_vel_[2];
   }
   if(side == "FR") {
   download_parameters.t_ff[3] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[3] = goal_angle_[0];
   download_parameters.goal_angle_vel[3] = goal_ang_vel_[0];
   download_parameters.t_ff[4] = kk1 * T1;
   download_parameters.goal_angle[4] = goal_angle_[1];
   download_parameters.goal_angle_vel[4] = goal_ang_vel_[1];
   download_parameters.t_ff[5] = kk2 * T2;
   download_parameters.goal_angle[5] = goal_angle_[2];
   download_parameters.goal_angle_vel[5] = goal_ang_vel_[2];
   }
   if(side == "HL") {
   download_parameters.t_ff[6] = -kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[6] = goal_angle_[0];
   download_parameters.goal_angle_vel[6] = goal_ang_vel_[0];
   download_parameters.t_ff[7] =kk1 *  T1;
   download_parameters.goal_angle[7] = goal_angle_[1];
   download_parameters.goal_angle_vel[7] = goal_ang_vel_[1];
   download_parameters.t_ff[8] = kk2 * T2;
   download_parameters.goal_angle[8]= goal_angle_[2];
   download_parameters.goal_angle_vel[8] = goal_ang_vel_[2];
   }
   if(side == "HR") {
   download_parameters.t_ff[9] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[9] = goal_angle_[0];
   download_parameters.goal_angle_vel[9] = goal_ang_vel_[0];
   download_parameters.t_ff[10] = kk1 * T1;
   download_parameters.goal_angle[10] = goal_angle_[1];
   download_parameters.goal_angle_vel[10] = goal_ang_vel_[1];
   download_parameters.t_ff[11] =kk2 *  T2;
   download_parameters.goal_angle[11] = goal_angle_[2];
   download_parameters.goal_angle_vel[11] = goal_ang_vel_[2];
   }
   
 //
	knee_vel_offset_ = goal_ang_vel_[2] - velocity_[2];
	knee_angle_offset_ = abs(goal_angle_[2] - angle_[2]);
	hipy_vel_offset_ = abs(goal_ang_vel_[1] - velocity_[1]);
	hipy_angle_offset_ = abs(goal_angle_[1] - angle_[1]);
}

void LegControl::SwingToPosition(Vec3 goal_pos, Vec3 goal_pos_next, Vec3 goal_pos_next2, int n, string side, double kp_rate)
{
	Vec3 goal_angle_next;
	Vec3 goal_angle_next2;

	switch (n) {
	case 0: {
		goal_angle_ = CalculateAngleLocal(goal_pos);
		goal_angle_next = CalculateAngleLocal(goal_pos_next);
		goal_angle_next2 =CalculateAngleLocal(goal_pos_next2);
		break;
	}
	case 2: {
		goal_angle_ = CalculateAngleConsiderPitch(goal_pos);
		goal_angle_next = CalculateAngleConsiderPitch(goal_pos_next);
		goal_angle_next2 = CalculateAngleConsiderPitch(goal_pos_next2);
		break;
	}
	case 1: {
		goal_angle_ = CalculateAngleGlobal(goal_pos);
		goal_angle_next = CalculateAngleGlobal(goal_pos_next);
		goal_angle_next2 = CalculateAngleGlobal(goal_pos_next2);
		break;
	}
	default: {
		goal_angle_ = CalculateAngleGlobal(goal_pos);
		goal_angle_next = CalculateAngleGlobal(goal_pos_next);
		goal_angle_next2 = CalculateAngleGlobal(goal_pos_next2);
		break;
	}
	}
	goal_ang_vel_ = (goal_angle_next - goal_angle_) / cycle_time_;
	goal_ang_vel_next_ = (goal_angle_next2 - goal_angle_next) / cycle_time_;
	goal_ang_acc_ = (goal_ang_vel_next_ - goal_ang_vel_) / cycle_time_;

	double k1, k2, k3;
	k1 = 0;
	k2 = 0.0;
	k3 = 1;
	double M_hipx = 0;//0.15;  //0.123
	Vec3 coulombs; Vec3 low_vel_compensation;coulombs << 0, 0, 0;low_vel_compensation << 0, 0, 0;
	if (side == "FR") {
		global_.fr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = kp_rate * control_parameter_.hipx_kp * (goal_angle_[0] - angle_[0]) + control_parameter_.hipx_kd * (goal_ang_vel_[0] - velocity_[0]) + 0 * global_.fr_hipx_integral
			+ M_hipx * goal_ang_acc_[0] +kk0 *  k3 * control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] + control_parameter_.gravity_coef3 * kDegree2Radian) + k1 * goal_ang_vel_[0] + k2 * Sign(goal_ang_vel_[0]);//+0.00756*9.5*kGravity*cos(angle_[0]);

		CoulombsCoefficient(coulombs, goal_ang_vel_);
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.3*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];

	} else if (side == "FL") {
		global_.fl_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = kp_rate * control_parameter_.hipx_kp * (goal_angle_[0] - angle_[0]) + control_parameter_.hipx_kd * (goal_ang_vel_[0] - velocity_[0]) + 0 * global_.fl_hipx_integral
			+ M_hipx * goal_ang_acc_[0] +kk0 *  k3 * control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] - control_parameter_.gravity_coef3 * kDegree2Radian) + k1 * goal_ang_vel_[0] + k2 * Sign(goal_ang_vel_[0]);//-0.00756*9.5*kGravity*cos(angle_[0]);

		CoulombsCoefficient(coulombs, goal_ang_vel_);
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.5*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];
	} else if (side == "HR") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = kp_rate * control_parameter_.hipx_kp * (goal_angle_[0] - angle_[0]) + control_parameter_.hipx_kd * (goal_ang_vel_[0] - velocity_[0]) + 0 * global_.hr_hipx_integral
			+ M_hipx * goal_ang_acc_[0] +kk0 *  k3 * control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] + control_parameter_.gravity_coef3 * kDegree2Radian) + k1 * goal_ang_vel_[0] + k2 * Sign(goal_ang_vel_[0]);//+0.00756*9.5*kGravity*cos(angle_[0]);

		CoulombsCoefficient(coulombs, goal_ang_vel_);
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 2 * coulombs[1];
		low_vel_compensation[2] = 2 * coulombs[2];
	} else if (side == "HL") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = kp_rate * control_parameter_.hipx_kp * (goal_angle_[0] - angle_[0]) + control_parameter_.hipx_kd * (goal_ang_vel_[0] - velocity_[0]) + 0 * global_.hr_hipx_integral
			+ M_hipx * goal_ang_acc_[0] +kk0 *  k3 * control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] - control_parameter_.gravity_coef3 * kDegree2Radian) + k1 * goal_ang_vel_[0] + k2 * Sign(goal_ang_vel_[0]);//-0.00756*9.5*kGravity*cos(angle_[0]);

		CoulombsCoefficient(coulombs, goal_ang_vel_);
		low_vel_compensation[0] = 3 * coulombs[0];
		low_vel_compensation[1] = 4 * coulombs[1];
		low_vel_compensation[2] = 0.75*coulombs[2];
	}

	double T1, T2, T0;
	DynamicsCompensation(goal_angle_, goal_angle_next, goal_angle_next2, T1, T2, T0);
	if (T1 >= 50) {
		T1 = 50;
	}
	if (T1 <= -50) {
		T1 = -50;
	}
	if (T2 >= 50) {
		T2 = 50;
	}
	if (T2 <= -50) {
		T2 = -50;
	}
	if (T0 >= 50) {
		T0 = 50;
	}
	if (T0 <= -50) {
		T0 = -50;
	}
 
	//	T0=0;T1=0;T2=0;
	//T0,T1,T2 is dynamic compensation,effect in high speed situation; low_vel_compensation is coulombs friction,effect in low speed situation
	torque_output_[1] = kp_rate * control_parameter_.hipy_kp * (goal_angle_[1] - angle_[1]) + control_parameter_.hipy_kd * (goal_ang_vel_[1] - velocity_[1]) + kk1 * T1 + 0*control_parameter_.low_vel * low_vel_compensation[1];
	torque_output_[2] = kp_rate * control_parameter_.knee_kp * (goal_angle_[2] - angle_[2]) + control_parameter_.knee_kd * (goal_ang_vel_[2] - velocity_[2]) + kk2 * T2 + 0*control_parameter_.low_vel * low_vel_compensation[2];
	torque_output_[0] = torque_output_[0] + T0 + 0*control_parameter_.low_vel * low_vel_compensation[0];
	knee_vel_offset_ = goal_ang_vel_[2] - velocity_[2];
	knee_angle_offset_ = abs(goal_angle_[2] - angle_[2]);
	hipy_vel_offset_ = abs(goal_ang_vel_[1] - velocity_[1]);
	hipy_angle_offset_ = abs(goal_angle_[1] - angle_[1]);
 	for(int i = 0; i < 4; i++){
		download_parameters.kp[3*i]=control_parameter_.hipx_kp;
		download_parameters.kp[3*i+1]=control_parameter_.hipy_kp;
		download_parameters.kp[3*i+2]=control_parameter_.knee_kp;
		download_parameters.kd[3*i]=control_parameter_.hipx_kd;
		download_parameters.kd[3*i+1]=control_parameter_.hipy_kd;
		download_parameters.kd[3*i+2]=control_parameter_.knee_kd;
	} 
   if(side == "FL") {
   download_parameters.t_ff[0] =  -kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[0] = goal_angle_[0];
   download_parameters.goal_angle_vel[0] = goal_ang_vel_[0];
   download_parameters.t_ff[1] = kk1*T1;
   download_parameters.goal_angle[1] = goal_angle_[1];
   download_parameters.goal_angle_vel[1] = goal_ang_vel_[1];
   download_parameters.t_ff[2] = kk2*T2;
   download_parameters.goal_angle[2] = goal_angle_[2];
   download_parameters.goal_angle_vel[2] = goal_ang_vel_[2];
   }
   if(side == "FR") {
   download_parameters.t_ff[3] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[3] = goal_angle_[0];
   download_parameters.goal_angle_vel[3] = goal_ang_vel_[0];
   download_parameters.t_ff[4] = kk1 * T1;
   download_parameters.goal_angle[4] = goal_angle_[1];
   download_parameters.goal_angle_vel[4] = goal_ang_vel_[1];
   download_parameters.t_ff[5] = kk2 * T2;
   download_parameters.goal_angle[5] = goal_angle_[2];
   download_parameters.goal_angle_vel[5] = goal_ang_vel_[2];
   }
   if(side == "HL") {
   download_parameters.t_ff[6] = -kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[6] = goal_angle_[0];
   download_parameters.goal_angle_vel[6] = goal_ang_vel_[0];
   download_parameters.t_ff[7] =kk1 *  T1;
   download_parameters.goal_angle[7] = goal_angle_[1];
   download_parameters.goal_angle_vel[7] = goal_ang_vel_[1];
   download_parameters.t_ff[8] = kk2 * T2;
   download_parameters.goal_angle[8]= goal_angle_[2];
   download_parameters.goal_angle_vel[8] = goal_ang_vel_[2];
   }
   if(side == "HR") {
   download_parameters.t_ff[9] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[9] = goal_angle_[0];
   download_parameters.goal_angle_vel[9] = goal_ang_vel_[0];
   download_parameters.t_ff[10] = kk1 * T1;
   download_parameters.goal_angle[10] = goal_angle_[1];
   download_parameters.goal_angle_vel[10] = goal_ang_vel_[1];
   download_parameters.t_ff[11] =kk2 *  T2;
   download_parameters.goal_angle[11] = goal_angle_[2];
   download_parameters.goal_angle_vel[11] = goal_ang_vel_[2];
   }
}

void LegControl::SwingToPosition(Vec3 goal_pos, Vec3 goal_pos_next, Vec3 goal_pos_next2, int n, string side, double kp_rate ,double kd_rate){
	Vec3 goal_angle_next;
	Vec3 goal_angle_next2;

	switch (n) {
	case 0: {
		goal_angle_ = CalculateAngleLocal(goal_pos);
		goal_angle_next = CalculateAngleLocal(goal_pos_next);
		goal_angle_next2 = CalculateAngleLocal(goal_pos_next2);
		break;
	}
	case 2: {
		goal_angle_ = CalculateAngleConsiderPitch(goal_pos);
		goal_angle_next = CalculateAngleConsiderPitch(goal_pos_next);
		goal_angle_next2 = CalculateAngleConsiderPitch(goal_pos_next2);
		break;
	}
	case 1: {
		goal_angle_ = CalculateAngleGlobal(goal_pos);
		goal_angle_next = CalculateAngleGlobal(goal_pos_next);
		goal_angle_next2 = CalculateAngleGlobal(goal_pos_next2);
		break;
	}
	default: {
		goal_angle_ = CalculateAngleGlobal(goal_pos);
		goal_angle_next = CalculateAngleGlobal(goal_pos_next);
		goal_angle_next2 = CalculateAngleGlobal(goal_pos_next2);
		break;
	}
	}
	goal_ang_vel_ = (goal_angle_next - goal_angle_) / cycle_time_;
	goal_ang_vel_next_ = (goal_angle_next2 - goal_angle_next) / cycle_time_;
	goal_ang_acc_ = (goal_ang_vel_next_ - goal_ang_vel_) / cycle_time_;

	double k1, k2, k3;
	k1 = 0;
	k2 = 0.0;
	k3 = 1;
	double M_hipx = 0;//0.15;  //0.123
	Vec3 coulombs; Vec3 low_vel_compensation;coulombs << 0, 0, 0;low_vel_compensation << 0, 0, 0;
	if (side == "FR") {
		global_.fr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = kp_rate * control_parameter_.hipx_kp * (goal_angle_[0] - angle_[0]) + kd_rate *control_parameter_.hipx_kd * (goal_ang_vel_[0] - velocity_[0]) + 0 * global_.fr_hipx_integral
			+ M_hipx * goal_ang_acc_[0] +kk0 *  k3 * control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] + control_parameter_.gravity_coef3 * kDegree2Radian) + k1 * goal_ang_vel_[0] + k2 * Sign(goal_ang_vel_[0]);//+0.00756*9.5*kGravity*cos(angle_[0]);

		CoulombsCoefficient(coulombs, goal_ang_vel_);
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.3*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];

	} else if (side == "FL") {
		global_.fl_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = kp_rate * control_parameter_.hipx_kp * (goal_angle_[0] - angle_[0]) + kd_rate *control_parameter_.hipx_kd * (goal_ang_vel_[0] - velocity_[0]) + 0 * global_.fl_hipx_integral
			+ M_hipx * goal_ang_acc_[0] +kk0 *  k3 * control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] - control_parameter_.gravity_coef3 * kDegree2Radian) + k1 * goal_ang_vel_[0] + k2 * Sign(goal_ang_vel_[0]);//-0.00756*9.5*kGravity*cos(angle_[0]);

		CoulombsCoefficient(coulombs, goal_ang_vel_);
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 1.5*coulombs[1];
		low_vel_compensation[2] = 1 * coulombs[2];
	} else if (side == "HR") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = kp_rate * control_parameter_.hipx_kp * (goal_angle_[0] - angle_[0]) + kd_rate *control_parameter_.hipx_kd * (goal_ang_vel_[0] - velocity_[0]) + 0 * global_.hr_hipx_integral
			+ M_hipx * goal_ang_acc_[0] +kk0 *  k3 * control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] + control_parameter_.gravity_coef3 * kDegree2Radian) + k1 * goal_ang_vel_[0] + k2 * Sign(goal_ang_vel_[0]);//+0.00756*9.5*kGravity*cos(angle_[0]);

		CoulombsCoefficient(coulombs, goal_ang_vel_);
		low_vel_compensation[0] = 1 * coulombs[0];
		low_vel_compensation[1] = 2 * coulombs[1];
		low_vel_compensation[2] = 2 * coulombs[2];
	} else if (side == "HL") {
		global_.hr_hipx_integral += goal_angle_[0] - angle_[0];
		torque_output_[0] = kp_rate * control_parameter_.hipx_kp * (goal_angle_[0] - angle_[0]) + kd_rate *control_parameter_.hipx_kd * (goal_ang_vel_[0] - velocity_[0]) + 0 * global_.hr_hipx_integral
			+ M_hipx * goal_ang_acc_[0] +kk0 *  k3 * control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * sin(angle_[0] - control_parameter_.gravity_coef3 * kDegree2Radian) + k1 * goal_ang_vel_[0] + k2 * Sign(goal_ang_vel_[0]);//-0.00756*9.5*kGravity*cos(angle_[0]);

		CoulombsCoefficient(coulombs, goal_ang_vel_);
		low_vel_compensation[0] = 3 * coulombs[0];
		low_vel_compensation[1] = 4 * coulombs[1];
		low_vel_compensation[2] = 0.75*coulombs[2];
	}

	double T1, T2, T0;
	DynamicsCompensation(goal_angle_, goal_angle_next, goal_angle_next2, T1, T2, T0);
	if (T1 >= 50) {
		T1 = 50;
	}
	if (T1 <= -50) {
		T1 = -50;
	}
	if (T2 >= 50) {
		T2 = 50;
	}
	if (T2 <= -50) {
		T2 = -50;
	}
	if (T0 >= 50) {
		T0 = 50;
	}
	if (T0 <= -50) {
		T0 = -50;
	}
	//	T0=0;T1=0;T2=0;
	//T0,T1,T2 is dynamic compensation,effect in high speed situation; low_vel_compensation is coulombs friction,effect in low speed situation
	torque_output_[1] = kp_rate * control_parameter_.hipy_kp * (goal_angle_[1] - angle_[1]) + kd_rate *control_parameter_.hipy_kd * (goal_ang_vel_[1] - velocity_[1]) + kk1 * T1 + 0*control_parameter_.low_vel * low_vel_compensation[1];
	torque_output_[2] = kp_rate * control_parameter_.knee_kp * (goal_angle_[2] - angle_[2]) + kd_rate *control_parameter_.knee_kd * (goal_ang_vel_[2] - velocity_[2]) + kk2 * T2 +0* control_parameter_.low_vel * low_vel_compensation[2];
	torque_output_[0] = torque_output_[0] + T0 + control_parameter_.low_vel * low_vel_compensation[0];
	knee_vel_offset_ = goal_ang_vel_[2] - velocity_[2];
	knee_angle_offset_ = abs(goal_angle_[2] - angle_[2]);
	hipy_vel_offset_ = abs(goal_ang_vel_[1] - velocity_[1]);
	hipy_angle_offset_ = abs(goal_angle_[1] - angle_[1]);
 	for(int i = 0; i < 12; i++){
		download_parameters.kp[i]=kp_rate;
	}
 	for(int i = 0; i < 4; i++){
		download_parameters.kd[3*i]=kd_rate*control_parameter_.hipx_kd;
		download_parameters.kd[3*i+1]=kd_rate*control_parameter_.hipy_kd;
		download_parameters.kd[3*i+2]=kd_rate*control_parameter_.knee_kd;
	}
   if(side == "FL") {
   download_parameters.t_ff[0] =  -kk0 * double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[0] = goal_angle_[0];
   download_parameters.goal_angle_vel[0] = goal_ang_vel_[0];
   download_parameters.t_ff[1] = kk1*T1;
   download_parameters.goal_angle[1] = goal_angle_[1];
   download_parameters.goal_angle_vel[1] = goal_ang_vel_[1];
   download_parameters.t_ff[2] = kk2*T2;
   download_parameters.goal_angle[2] = goal_angle_[2];
   download_parameters.goal_angle_vel[2] = goal_ang_vel_[2];
   }
   if(side == "FR") {
   download_parameters.t_ff[3] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[3] = goal_angle_[0];
   download_parameters.goal_angle_vel[3] = goal_ang_vel_[0];
   download_parameters.t_ff[4] = kk1 * T1;
   download_parameters.goal_angle[4] = goal_angle_[1];
   download_parameters.goal_angle_vel[4] = goal_ang_vel_[1];
   download_parameters.t_ff[5] = kk2 * T2;
   download_parameters.goal_angle[5] = goal_angle_[2];
   download_parameters.goal_angle_vel[5] = goal_ang_vel_[2];
   }
   if(side == "HL") {
   download_parameters.t_ff[6] = -kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[6] = goal_angle_[0];
   download_parameters.goal_angle_vel[6] = goal_ang_vel_[0];
   download_parameters.t_ff[7] =kk1 *  T1;
   download_parameters.goal_angle[7] = goal_angle_[1];
   download_parameters.goal_angle_vel[7] = goal_ang_vel_[1];
   download_parameters.t_ff[8] = kk2 * T2;
   download_parameters.goal_angle[8]= goal_angle_[2];
   download_parameters.goal_angle_vel[8] = goal_ang_vel_[2];
   }
   if(side == "HR") {
   download_parameters.t_ff[9] = kk0 *  double(k3* control_parameter_.gravity_coef1 * kGravity * control_parameter_.gravity_coef2 * cos(angle_[0])) + T0;
   download_parameters.goal_angle[9] = goal_angle_[0];
   download_parameters.goal_angle_vel[9] = goal_ang_vel_[0];
   download_parameters.t_ff[10] = kk1 * T1;
   download_parameters.goal_angle[10] = goal_angle_[1];
   download_parameters.goal_angle_vel[10] = goal_ang_vel_[1];
   download_parameters.t_ff[11] =kk2 *  T2;
   download_parameters.goal_angle[11] = goal_angle_[2];
   download_parameters.goal_angle_vel[11] = goal_ang_vel_[2];
   }
}

void LegControl::SwingToPositionHipY(double goal_angle, string side)
{
	double goal_angle_ = goal_angle;

	double goal_ang_vel_ = 0;
	double goal_ang_acc_ = 0;

	double k1, k2;
	k1 = 0;
	k2 = 0.0;
	double M_hipx = 0.15;  //0.123

	if (side == "FR") {
		global_.fr_hipx_integral += goal_angle_ - angle_[0];
		torque_output_[0] = control_parameter_.hipx_kp * (goal_angle_ - angle_[0]) + control_parameter_.hipx_kd * (goal_ang_vel_ - velocity_[0]) + 0 * global_.fr_hipx_integral
				            + M_hipx * goal_ang_acc_ + 10 * kGravity * 0.02922 * sin(angle_[0] + 27.38 * kDegree2Radian) + k1 * goal_ang_vel_ + k2 * Sign(goal_ang_vel_);
	} else if (side == "FL") {
		global_.fl_hipx_integral += goal_angle_ - angle_[0];
		torque_output_[0] = control_parameter_.hipx_kp * (goal_angle_ - angle_[0]) + control_parameter_.hipx_kd * (goal_ang_vel_ - velocity_[0]) + 0 * global_.fl_hipx_integral
			                + M_hipx * goal_ang_acc_ + 10 * kGravity * 0.02922 * sin(angle_[0] - 27.38 * kDegree2Radian) + k1 * goal_ang_vel_ + k2 * Sign(goal_ang_vel_);
	} else if (side == "HR") {
		global_.hr_hipx_integral += goal_angle_ - angle_[0];
		torque_output_[0] = control_parameter_.hipx_kp * (goal_angle_ - angle_[0]) + control_parameter_.hipx_kd * (goal_ang_vel_ - velocity_[0]) + 0 * global_.hr_hipx_integral
			                + M_hipx * goal_ang_acc_ + 10 * kGravity * 0.02922 * sin(angle_[0] + 27.38 * kDegree2Radian) + k1 * goal_ang_vel_ + k2 * Sign(goal_ang_vel_);
	} else if (side == "HL") {
		global_.hr_hipx_integral += goal_angle_ - angle_[0];
		torque_output_[0] = control_parameter_.hipx_kp * (goal_angle_ - angle_[0]) + control_parameter_.hipx_kd * (goal_ang_vel_ - velocity_[0]) + 0 * global_.hr_hipx_integral
				            + M_hipx * goal_ang_acc_ + 10 * kGravity * 0.02922 * sin(angle_[0] - 27.38 * kDegree2Radian) + k1 * goal_ang_vel_ + k2 * Sign(goal_ang_vel_);
	}
}

void LegControl::ForwardKinematic(const Vec3 &angle, const Vec3 &ang_vel, Vec3 *calculate_xyz, Vec3 *calculate_xyz_vel)
{
	double l0, l1, l2;
	double s1, s2, s3;
	double ds1, ds2, ds3;
	double x, y, z, zt;
	double dx, dy, dzt, dz;

	l0 = link_.link_length[0]; l1 = link_.link_length[1]; l2 = link_.link_length[2];
	s1 = angle[0]; s2 = angle[1]; s3 = angle[2];
	ds1 = ang_vel[0]; ds2 = ang_vel[1]; ds3 = ang_vel[2];

	// using the hip joint as the origin of coordinates, the axis direction is the same as the body coordination.
	x = l1 * sin(s2) + l2 * sin(s2 + s3);
	dx = l1 * cos(s2) * ds2 + l2 * cos(s2 + s3) * (ds2 + ds3);

	zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
	dzt = l1 * sin(s2) * ds2 + l2 * sin(s2 + s3) * (ds2 + ds3);

	y = zt * sin(s1) - l0 * cos(s1);
	dy = dzt * sin(s1) + zt * cos(s1) * ds1 + l0 * sin(s1) * ds1;

	z = zt * cos(s1) + l0 * sin(s1);
	dz = dzt * cos(s1) - zt * sin(s1) * ds1 + l0 * cos(s1) * ds1;

	(*calculate_xyz) << x, y, z;
	(*calculate_xyz_vel) << dx, dy, dz;
}

void LegControl::ForwardKinematicLowPassFilter(Vec3 &angle, Vec3 &ang_vel, Vec3 &calculate_xyz, Vec3 &calculate_xyz_vel, string side)
{
	double l0, l1, l2;
	double s1, s2, s3;
	double ds1, ds2, ds3;
	double x, y, z, zt;
	double dx, dy, dzt, dz;

	l0 = link_.link_length[0]; l1 = link_.link_length[1]; l2 = link_.link_length[2];
	s1 = angle[0]; s2 = angle[1]; s3 = angle[2];
	/*if (side == "FL") {
		ds1 = ang_vel[0]; ds2 = global_->fl_hipy_vel_filter; ds3 = global_->fl_knee_vel_filter;
		s1 = angle[0]; s2 = global_->fl_hipy_pos_filter; s3 = global_->fl_knee_pos_filter;
	}
	else if (side == "FR") {
		ds1 = ang_vel[0]; ds2 = global_->fr_hipy_vel_filter; ds3 = global_->fr_knee_vel_filter;
		s1 = angle[0]; s2 = global_->fr_hipy_pos_filter; s3 = global_->fr_knee_pos_filter;
	}
	else if (side == "HL") {
		ds1 = ang_vel[0]; ds2 = global_->hl_hipy_vel_filter; ds3 = global_->hl_knee_vel_filter;
		s1 = angle[0]; s2 = global_->hl_hipy_pos_filter; s3 = global_->hl_knee_pos_filter;
	}
	else if (side == "HR") {
		ds1 = ang_vel[0]; ds2 = global_->hr_hipy_vel_filter; ds3 = global_->hr_knee_vel_filter;
		s1 = angle[0]; s2 = global_->hr_hipy_pos_filter; s3 = global_->hr_knee_pos_filter;
		//AddToScope(global_->hr_knee_filter, 37);
		//ds1 = ang_vel[0]; ds2 = ang_vel[1]; ds3 = global_->hr_knee_filter;
	}*/
	ds1 = ang_vel[0]; ds2 = ang_vel[1]; ds3 = ang_vel[2];

	// using the hip joint as the origin of coordinates, the axis direction is the same as the body coordination.
	x = l1 * sin(s2) + l2 * sin(s2 + s3);
	dx = l1 * cos(s2) * ds2 + l2 * cos(s2 + s3) * (ds2 + ds3);

	zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
	dzt = l1 * sin(s2) * ds2 + l2 * sin(s2 + s3) * (ds2 + ds3);

	y = zt * sin(s1) - l0 * cos(s1);
	dy = dzt * sin(s1) + zt * cos(s1) * ds1 + l0 * sin(s1) * ds1;

	z = zt * cos(s1) + l0 * sin(s1);
	dz = dzt * cos(s1) - zt * sin(s1) * ds1 + l0 * cos(s1) * ds1;

	calculate_xyz << x, y, z;
	calculate_xyz_vel << dx, dy, dz;
}

void LegControl::ForwardKinematic(Vec3 *calculate_xyz, Vec3 *calculate_xyz_vel)
{
	ForwardKinematic(angle_, velocity_, calculate_xyz, calculate_xyz_vel);
}

void LegControl::ForwardKinematicXYZ(Vec3 *calculate_xyz)
{
	Vec3 tmp;

	tmp << 0, 0, 0;

	ForwardKinematic(angle_, tmp, calculate_xyz, &tmp);
}

void LegControl::ForwardKinematicCoordinateTransform(Vec3 *calculate_xyz)
{
	ForwardKinematicXYZ(calculate_xyz);
	*calculate_xyz = -*calculate_xyz;
}

void LegControl::TorqueOutputLimit()
{

}

void LegControl::VirtualForceToTorqueOutput(Vec3 force_xyz_global)
{
	double l0 = link_.link_length[0];
	double l1 = link_.link_length[1];
	double l2 = link_.link_length[2];
	double s1 = angle_[0];
	double s2 = angle_[1];
	double s3 = angle_[2];

	double tor_hipx, tor_hipy, tor_knee;
	double force_x_local, force_y_local, force_z_local;
	double zt;
	double force_x_global_tmp = force_xyz_global[0];
	double force_y_global_tmp = force_xyz_global[1];

	Vec3 body_rpy;
	Vec3 force_transform;

	// prevent the leg get off the ground.
	if (force_xyz_global[2] < kMinFz) {
		force_xyz_global[2] = kMinFz;
	}

	GetAbsMin(force_x_global_tmp, kFrictionalCoefficientTrot * force_xyz_global[2], "virtual_force_x_global");
	GetAbsMin(force_y_global_tmp, kFrictionalCoefficientTrot * force_xyz_global[2], "virtual_force_y_global");

	GetAbsMin(force_x_global_tmp, kMaxFx, "virtual_force_x_global");
	GetAbsMin(force_y_global_tmp, kMaxFy, "virtual_force_y_global");

	force_xyz_global[0] = force_x_global_tmp;
	force_xyz_global[1] = force_y_global_tmp;

	zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
	//	s1t = s1 - asin(l0/zt);

	//Body_RPY << Gyro->Rol, Gyro->Pit, Gyro->Yaw;
	body_rpy << gyro_data_.roll, gyro_data_.pitch, 0.0;
	force_transform = VirtualForceCoordinateTransformFrom(force_xyz_global, body_rpy, s1);
	force_x_local = force_transform[0]; force_y_local = force_transform[1]; force_z_local = force_transform[2];
	virtual_force_local_[0] = force_x_local;
	virtual_force_local_[1] = force_y_local;
	virtual_force_local_[2] = force_z_local;

	tor_knee = -force_x_local * l2 * cos(s2 + s3) - force_z_local * l2 * sin(s2 + s3);
	tor_hipy = tor_knee - force_z_local * l1 * sin(s2) - force_x_local * l1 * cos(s2);
	tor_hipx = -force_y_local * zt - force_z_local * l0;

	torque_output_[0] = tor_hipx; torque_output_[1] = tor_hipy; torque_output_[2] = tor_knee;
// if(leg_id == 0){
	for(int i = 0; i < 12; i++){
		download_parameters.kp[i]=0;
		download_parameters.kd[i]=0;
		//download_parameters.t_ff[i]=0;
	}   
if(leg_id == 0){
    download_parameters.goal_angle[0] = angle_[0];
    download_parameters.goal_angle_vel[0] = velocity_[0];
    download_parameters.goal_angle[1] = angle_[1];
    download_parameters.goal_angle_vel[1] = velocity_[1];
    download_parameters.goal_angle[2] = angle_[2];
    download_parameters.goal_angle_vel[2] = velocity_[2];
    download_parameters.t_ff[0] =tor_hipx;
    download_parameters.t_ff[1] =tor_hipy;
    download_parameters.t_ff[2] =tor_knee;
    AddToScope(tor_knee,55 ,"");
    }
if(leg_id == 1){
    download_parameters.goal_angle[3] = angle_[0];
    download_parameters.goal_angle_vel[3] = velocity_[0];
    download_parameters.goal_angle[4] = angle_[1];
    download_parameters.goal_angle_vel[4] = velocity_[1];
    download_parameters.goal_angle[5] = angle_[2];
    download_parameters.goal_angle_vel[5] = velocity_[2];
    download_parameters.t_ff[3] = tor_hipx;
    download_parameters.t_ff[4] = tor_hipy;
    download_parameters.t_ff[5] =tor_knee;}
if(leg_id == 2){
    download_parameters.goal_angle[6] = angle_[0];
    download_parameters.goal_angle_vel[6] = velocity_[0];
    download_parameters.goal_angle[7] = angle_[1];
    download_parameters.goal_angle_vel[7] = velocity_[1];
    download_parameters.goal_angle[8] = angle_[2];
    download_parameters.goal_angle_vel[8] = velocity_[2];
    download_parameters.t_ff[6] = tor_hipx;
    download_parameters.t_ff[7] = tor_hipy;
    download_parameters.t_ff[8] = tor_knee;}
if(leg_id == 3){
    download_parameters.goal_angle[9] = angle_[0];
    download_parameters.goal_angle_vel[9] = velocity_[0];
    download_parameters.goal_angle[10] = angle_[1];
    download_parameters.goal_angle_vel[10] = velocity_[1];
    download_parameters.goal_angle[11] = angle_[2];
    download_parameters.goal_angle_vel[11] = velocity_[2];
    download_parameters.t_ff[9] = tor_hipx;
    download_parameters.t_ff[10] = tor_hipy;
    download_parameters.t_ff[11] = tor_knee;}
// }

}

void LegControl::VirtualForceToTorqueOutput(Vec3 force_xyz, double Fz_min)
{
	double l0 = link_.link_length[0];
	double l1 = link_.link_length[1];
	double l2 = link_.link_length[2];
	double s1 = angle_[0];
	double s2 = angle_[1];
	double s3 = angle_[2];

	double tor1, tor2, tor3;
	double force_x, force_y, force_z;
	double zt;

	Vec3 body_rpy;
	Vec3 force_transform;

	// prevent the leg get off the ground.
	if (force_xyz[2] < Fz_min)
		force_xyz[2] = Fz_min;

	//double force_x_tmp = force_xyz[0];
	//GetAbsMin(force_x_tmp, (double)kFrictionalCoefficient*force_xyz[2]);
	//GetAbsMin(force_x_tmp, kMaxFx);
	//force_xyz[0] = force_x_tmp;

	zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
	//	s1t = s1 - asin(l0/zt);

	//Body_RPY << Gyro->Rol, Gyro->Pit, Gyro->Yaw;
	body_rpy << gyro_data_.roll, gyro_data_.pitch, 0.0;
	force_transform = VirtualForceCoordinateTransformFrom(force_xyz, body_rpy, s1);
	force_x = force_transform[0]; force_y = force_transform[1]; force_z = force_transform[2];

	tor3 = -force_x * l2 * cos(s2 + s3) - force_z * l2 * sin(s2 + s3);
	tor2 = tor3 - force_z * l1 * sin(s2) - force_x * l1 * cos(s2);
	tor1 = -force_y * zt - force_z * l0;

	torque_output_[0] = tor1; torque_output_[1] = tor2; torque_output_[2] = tor3;
}

void LegControl::VirtualForceToTorqueOutputPronk(Vec3 force_xyz_global, double FricCoef)
{
	double l0 = link_.link_length[0];
	double l1 = link_.link_length[1];
	double l2 = link_.link_length[2];
	double s1 = angle_[0];
	double s2 = angle_[1];
	double s3 = angle_[2];

	double tor_hipx, tor_hipy, tor_knee;
	double force_x_local, force_y_local, force_z_local;
	double zt;
	double force_x_global_tmp = force_xyz_global[0];
	double force_y_global_tmp = force_xyz_global[1];

	Vec3 body_rpy;
	Vec3 force_transform;

	// prevent the leg get off the ground.
	if (force_xyz_global[2] < kMinFz) {
		force_xyz_global[2] = kMinFz;
	}

	GetAbsMin(force_x_global_tmp, FricCoef * force_xyz_global[2], "virtual_force_x_global");
	GetAbsMin(force_y_global_tmp, FricCoef * force_xyz_global[2], "virtual_force_y_global");

	GetAbsMin(force_x_global_tmp, kMaxFx, "virtual_force_x_global");
	GetAbsMin(force_y_global_tmp, kMaxFy, "virtual_force_y_global");

	force_xyz_global[0] = force_x_global_tmp;
	force_xyz_global[1] = force_y_global_tmp;

	zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
	//	s1t = s1 - asin(l0/zt);

	//Body_RPY << Gyro->Rol, Gyro->Pit, Gyro->Yaw;
	body_rpy << gyro_data_.roll, gyro_data_.pitch, 0.0;
	force_transform = VirtualForceCoordinateTransformFrom(force_xyz_global, body_rpy, s1);
	force_x_local = force_transform[0]; force_y_local = force_transform[1]; force_z_local = force_transform[2];
	virtual_force_local_[0] = force_x_local;
	virtual_force_local_[1] = force_y_local;
	virtual_force_local_[2] = force_z_local;

	tor_knee = -force_x_local * l2 * cos(s2 + s3) - force_z_local * l2 * sin(s2 + s3);
	tor_hipy = tor_knee - force_z_local * l1 * sin(s2) - force_x_local * l1 * cos(s2);
	tor_hipx = -force_y_local * zt - force_z_local * l0;

	torque_output_[0] = tor_hipx; torque_output_[1] = tor_hipy; torque_output_[2] = tor_knee;
	//AddToScope( torque_output_[2], 119, "tor_knee");

	// if(leg_id == 0){
	for(int i = 0; i < 12; i++){
		download_parameters.kp[i]=0;
		download_parameters.kd[i]=0;
		//download_parameters.t_ff[i]=0;
	}
if(leg_id == 0){
    download_parameters.goal_angle[0] = angle_[0];
    download_parameters.goal_angle_vel[0] = velocity_[0];
    download_parameters.goal_angle[1] = angle_[1];
    download_parameters.goal_angle_vel[1] = velocity_[1];
    download_parameters.goal_angle[2] = angle_[2];
    download_parameters.goal_angle_vel[2] = velocity_[2];
    download_parameters.t_ff[0] =tor_hipx;
    download_parameters.t_ff[1] =tor_hipy;
    download_parameters.t_ff[2] =tor_knee;
    AddToScope(tor_knee,55 ,"");
    }
if(leg_id == 1){
    download_parameters.goal_angle[3] = angle_[0];
    download_parameters.goal_angle_vel[3] = velocity_[0];
    download_parameters.goal_angle[4] = angle_[1];
    download_parameters.goal_angle_vel[4] = velocity_[1];
    download_parameters.goal_angle[5] = angle_[2];
    download_parameters.goal_angle_vel[5] = velocity_[2];
    download_parameters.t_ff[3] = tor_hipx;
    download_parameters.t_ff[4] = tor_hipy;
    download_parameters.t_ff[5] =tor_knee;}
if(leg_id == 2){
    download_parameters.goal_angle[6] = angle_[0];
    download_parameters.goal_angle_vel[6] = velocity_[0];
    download_parameters.goal_angle[7] = angle_[1];
    download_parameters.goal_angle_vel[7] = velocity_[1];
    download_parameters.goal_angle[8] = angle_[2];
    download_parameters.goal_angle_vel[8] = velocity_[2];
    download_parameters.t_ff[6] = tor_hipx;
    download_parameters.t_ff[7] = tor_hipy;
    download_parameters.t_ff[8] = tor_knee;}
if(leg_id == 3){
    download_parameters.goal_angle[9] = angle_[0];
    download_parameters.goal_angle_vel[9] = velocity_[0];
    download_parameters.goal_angle[10] = angle_[1];
    download_parameters.goal_angle_vel[10] = velocity_[1];
    download_parameters.goal_angle[11] = angle_[2];
    download_parameters.goal_angle_vel[11] = velocity_[2];
    download_parameters.t_ff[9] = tor_hipx;
    download_parameters.t_ff[10] = tor_hipy;
    download_parameters.t_ff[11] = tor_knee;}
}

void LegControl::CalculateActualVirtualForce(Vec3 &force_xyz)
{
	double l0 = link_.link_length[0];
	double l1 = link_.link_length[1];
	double l2 = link_.link_length[2];
	double s1 = angle_[0];
	double s2 = angle_[1];
	double s3 = angle_[2];
	double lx1, lz1, lx2, lz2;

	Vec3 force_tmp, body_ryp;

	lx2 = l2 * cos(s2 + s3);
	lz2 = l2 * sin(s2 + s3);
	lx1 = l1 * cos(s2) + lx2;
	lz1 = l1 * sin(s2) + lz2;

	Matrix< double, 3, 3> pls;
	pls << 0, lx1, -l0, -lx1, 0, -lz1, -lx2, 0, -lz2;

	//force_tmp << pls.inverse() * torque_output_;
	body_ryp << gyro_data_.roll, gyro_data_.pitch, gyro_data_.yaw;
	//	s1t = s1 + asin(l0/lx1);
	force_xyz = VirtualForceCoordinateTransformTo(force_tmp, body_ryp, s1);
}

Vec3 LegControl::CalculateToeAngles()
{
	Vec3 toe_angles;
	Matrix3d Body_RM;

	double q1, q2, q3, q4, q5, q6, q345, s1, c1, s2, c2;
	double r11, r12, r13, r21; //r22, r23;
	double r31, r32, r33;

	q4 = angle_[2];
	q5 = angle_[1];
	q6 = angle_[0];

	Vec3 RPYs;
	RPYs <<gyro_data_.roll, gyro_data_.pitch, 0;
	Body_RM = RPY_to_RM(RPYs);        //  ??????????????
	r11 = Body_RM(0,0);
	r12 = Body_RM(0,1);
	r13 = Body_RM(0,2);

	r21 = Body_RM(1,0);
	//r22 = Body_RM(1,1);
	//r23 = Body_RM(1,2);

	r31 = Body_RM(2,0);
	r32 = Body_RM(2,1);
	r33 = Body_RM(2,2);

	q345 = atan2( -r31 , (r32*sin(q6) + r33*cos(q6)) );
	q3 = q345 - q4 - q5;                               // ????Pitch??

	s2 = cos(q6)*r32 - sin(q6)*r33;
	c2 = (sin(q6)*r32 + cos(q6)*r33) / cos(q345);
	q2 = atan2(s2, c2);                        // ????Roll??

	s1 = (sin(q345)*cos(q345) - sin(q6)*r11*r12 - cos(q6)*r11*r13) / (sin(q345)*r21 - sin(q2)*cos(q345)*r11);
	c1 = (sin(q2)*cos(q345)*cos(q345) - sin(q6)*r12*r21 - cos(q6)*r13*r21) / (sin(q2)*cos(q345)*r11 - sin(q345)*r21);

	q1 = atan2(s1, c1);                        // ????Yaw??
	//q1 = 0;                                          //  ????????????NaN??

	toe_angles << q1, q2, q3;

	return toe_angles;
}

Vec66 LegControl::CalculateJacobian()
{
	Matrix4d T01, T12, T23, T34, T45, T56;
	Matrix4d T02, T03, T04, T05, T06;
	Matrix3d R01, R02, R03, R04, R05, R06;
	Vector3d p1, p2, p3, p4, p5, p6;
	Vector3d aa1, aa2, aa3, aa4, aa5, aa6;
	Vector3d a1, a2, a3, a4, a5, a6;
	Vec6 j1, j2, j3, j4, j5, j6;
	Vec66 jacobian;

	double q1 = toe_angles_[0] * 0;         //?yaw??? ??Z??
	double q2 = toe_angles_[1];            // ?roll??? ??X??
	double q3 = toe_angles_[2];            // ?pitch??? ??Y??

	double q4 = angle_[2];                   // ???? ??Y?? ?????? ??Y??
	double q5 = angle_[1];                   // ??pitch??? ??Y??
	double q6 = angle_[0];                   // ??roll??? ??X??

	double L1 = link_.link_length[2];            // ??????????
	double L2 = link_.link_length[1];            // ?????????
	double Lhip = link_.link_length[0];      // ??HipX??HipY????

	//// ??????????? ////
	T01 << cos(q1), -sin(q1), 0, 0, sin(q1), cos(q1), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;         // ?yaw??? ??Z??
	T12 << 1, 0, 0, 0, 0, cos(q2), -sin(q2), 0, 0, sin(q2), cos(q2), 0, 0, 0, 0, 1;         // ?roll??? ??X??
	T23 << cos(q3), 0, sin(q3), 0, 0, 1, 0, 0, -sin(q3), 0, cos(q3), 0, 0, 0, 0, 1;         // ?pitch??? ??Y??
	T34 << cos(q4), 0, sin(q4), 0, 0, 1, 0, 0, -sin(q4), 0, cos(q4), L1, 0, 0, 0, 1;       // ???? ??Y??
	T45 << cos(q5), 0, sin(q5), 0, 0, 1, 0, 0, -sin(q5), 0, cos(q5), L2, 0, 0, 0, 1;       // ??pitch??? ??Y??
	T56<< 1, 0, 0, 0, 0, cos(q6), -sin(q6), Lhip, 0, sin(q6), cos(q6), 0, 0, 0, 0, 1;     // ??roll??? ??X??
	//T67 = [ 1 0 0 Lhx; 0 1 0 Lhy; 0 0 1 Lhz; 0 0 0 1];
	//T67 = T67*T78;

	//////////
	T02 = T01*T12;                          //  ?????????
	T03 = T01*T12*T23;
	T04 = T01*T12*T23*T34;
	T05 = T01*T12*T23*T34*T45;
	T06 = T01*T12*T23*T34*T45*T56;

	//// ???????????? ////
	R01 << T01(0,0),T01(0,1),T01(0,2),  T01(1,0),T01(1,1),T01(1,2),  T01(2,0),T01(2,1),T01(2,2);
	R02 << T02(0,0),T02(0,1),T02(0,2),  T02(1,0),T02(1,1),T02(1,2),  T02(2,0),T02(2,1),T02(2,2);
	R03 << T03(0,0),T03(0,1),T03(0,2),  T03(1,0),T03(1,1),T03(1,2),  T03(2,0),T03(2,1),T03(2,2);
	R04 << T04(0,0),T04(0,1),T04(0,2),  T04(1,0),T04(1,1),T04(1,2),  T04(2,0),T04(2,1),T04(2,2);
	R05 << T05(0,0),T05(0,1),T05(0,2),  T05(1,0),T05(1,1),T05(1,2),  T05(2,0),T05(2,1),T05(2,2);
	R06 << T06(0,0),T06(0,1),T06(0,2),  T06(1,0),T06(1,1),T06(1,2),  T06(2,0),T06(2,1),T06(2,2);

	// ????????? //
	p1 << T01(0,3), T01(1,3), T01(2,3);
	p2 << T02(0,3), T02(1,3), T02(2,3);
	p3 << T03(0,3), T03(1,3), T03(2,3);
	p4 << T04(0,3), T04(1,3), T04(2,3);
	p5 << T05(0,3), T05(1,3), T05(2,3);
	p6 << T06(0,3), T06(1,3), T06(2,3);

	//// ??????????? ////
	aa1 << 0, 0, 1;          // ??Z??
	aa2 << 1, 0, 0;          // ??X??
	aa3 << 0, 1, 0;          // ??Y??
	aa4 << 0, 1, 0;          // ??Y??
	aa5 << 0, 1, 0;          // ??Y??
	aa6 << 1, 0, 0;          // ??X??

	// ??????????? //
	a1 = R01*aa1;
	a2 = R02*aa2;
	a3 = R03*aa3;
	a4 = R04*aa4;
	a5 = R05*aa5;
	a6 = R06*aa6;

	j1 << (a1.cross(p6-p1)), a1;
	j2 << (a2.cross(p6-p2)), a2;
	j3 << (a3.cross(p6-p3)), a3;
	j4 << (a4.cross(p6-p4)), a4;
	j5 << (a5.cross(p6-p5)), a5;
	j6 << (a6.cross(p6-p6)), a6;

	jacobian << j1, j2, j3, j4, j5, j6;
	return jacobian;
}

Vec6 LegControl::CalculateHipStatesGlobal()
{
	Matrix4d T01, T12, T23, T34, T45, T56;
	Matrix4d T06;
	Vector3d p6;
	Vec33 B, D;
	Matrix3d A, C;
	Vector3d Vs;
	Vec6 hip_states;
	Vec3 RPYvs;
	RPYvs <<gyro_data_.rol_vel, gyro_data_.pit_vel, 0;
	//RPYvs(2) = 0;

	double q1 = toe_angles_[0]*0;         //?yaw??? ??Z??
	double q2 = toe_angles_[1];            // ?roll??? ??X??
	double q3 = toe_angles_[2];            // ?pitch??? ??Y??

	double q4 = angle_[2];                   // ???? ??Y?? ?????? ??Y??
	double q5 = angle_[1];                   // ??pitch??? ??Y??
	double q6 = angle_[0];                   // ??roll??? ??X??

	double qv4 = velocity_[2];                  // ???? ??Y?? ?????? ??Y??
	double qv5 = velocity_[1];                  // ??pitch??? ??Y??
	double qv6 = velocity_[0];                  // ??roll??? ??X??

	double L1 = link_.link_length[2];            // ??????????
	double L2 = link_.link_length[1];            // ?????????
	double Lhip = link_.link_length[0];       // ??HipX??HipY????

	//// ??????????? ////
	T01 << cos(q1), -sin(q1), 0, 0, sin(q1), cos(q1), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;         // ?yaw??? ??Z??
	T12 << 1, 0, 0, 0, 0, cos(q2), -sin(q2), 0, 0, sin(q2), cos(q2), 0, 0, 0, 0, 1;         // ?roll??? ??X??
	T23 << cos(q3), 0, sin(q3), 0, 0, 1, 0, 0, -sin(q3), 0, cos(q3), 0, 0, 0, 0, 1;         // ?pitch??? ??Y??
	T34 << cos(q4), 0, sin(q4), 0, 0, 1, 0, 0, -sin(q4), 0, cos(q4), L1, 0, 0, 0, 1;       // ???? ??Y??
	T45 << cos(q5), 0, sin(q5), 0, 0, 1, 0, 0, -sin(q5), 0, cos(q5), L2, 0, 0, 0, 1;       // ??pitch??? ??Y??
	T56<< 1, 0, 0, 0, 0, cos(q6), -sin(q6), Lhip, 0, sin(q6), cos(q6), 0, 0, 0, 0, 1;     // ??roll??? ??X??
	T06 = T01*T12*T23*T34*T45*T56;
	p6 << T06(0,3), T06(1,3), T06(2,3);

	A << jacobian_(0,0), jacobian_(0,1), jacobian_(0,2),   jacobian_(1,0), jacobian_(1,1), jacobian_(1,2),   jacobian_(2,0), jacobian_(2,1), jacobian_(2,2);
	C << jacobian_(3,0), jacobian_(3,1), jacobian_(3,2),   jacobian_(4,0), jacobian_(4,1), jacobian_(4,2),   jacobian_(5,0), jacobian_(5,1), jacobian_(5,2);

	B << jacobian_(0,3), jacobian_(0,4), jacobian_(0,5),   jacobian_(1,3), jacobian_(1,4), jacobian_(1,5),   jacobian_(2,3), jacobian_(2,4), jacobian_(2,5);
	D << jacobian_(3,3), jacobian_(3,4), jacobian_(3,5),   jacobian_(4,3), jacobian_(4,4), jacobian_(4,5),   jacobian_(5,3), jacobian_(5,4), jacobian_(5,5);

	Vector3d M3;
	M3 << qv4, qv5, qv6;

	Vs = A*(C.inverse())*RPYvs + (B - (A*(C.inverse())*D))*M3;
	hip_states << p6, Vs;

	return hip_states;
}

Vec6 LegControl::CalculateHipStatesLocal()
{
	Vec3 calculate_xyz, calculate_xyz_vel;
	Vec6 hip_states;
	ForwardKinematic(angle_, velocity_, &calculate_xyz, &calculate_xyz_vel);
	hip_states << -calculate_xyz[0], -calculate_xyz[1], -calculate_xyz[2], -calculate_xyz_vel[0], -calculate_xyz_vel[1], -calculate_xyz_vel[2];
	return hip_states;

}

void LegControl::DynamicsCompensation(Vec3 goal_angle_, Vec3 goal_angle_next, Vec3 goal_angle_next2, double &T1, double &T2, double &T0) {
	goal_ang_vel_ = (goal_angle_next - goal_angle_) / cycle_time_;
	goal_ang_vel_next_ = (goal_angle_next2 - goal_angle_next) / cycle_time_;
	goal_ang_acc_ = (goal_ang_vel_next_ - goal_ang_vel_) / cycle_time_;
	double l1 = link_.link_length[1];
	double l2 = link_.link_length[2];
	double s2 = angle_[1];
	double s3 = angle_[2];
	double v1 = goal_ang_vel_[1];
	double v2 = goal_ang_vel_[2];

	if (robot_id_ == "JY-A" || robot_id_ == "JY-B") {
		double d1 = 0.028;double d2 = 0.123;double m1 = 4.98;double m2 = 0.4;
		double I1 = 0.08;double I2 = 0.0425;double g = 9.8;double I0 = 0.122;
		T1 = goal_ang_acc_[1] * (I1 + m1 * d1*d1 + I2 + m2 * l1*l1 + m2 * d2*d2 + 2 * m2*l1*d2*cos(s3)) + goal_ang_acc_[2] * (I2 + m2 * d2*d2 + m2 * l1*d2*cos(s3)) 
			- 2 * m2*l1*d2*v1*v2 - m2 * l1*d2*v2*v2*sin(s3) + m1 * g*d1*sin(s2) + m2 * g*l1*sin(s2) + m2 * g*d2*sin(s2 + s3);
		T2 = goal_ang_acc_[1] * (I2 + m2 * d2*d2 + m2 * l1*d2*cos(s3)) + goal_ang_acc_[2] * (I2 + m2 * d2*d2) 
			+ v1 * v2*(m2*l1*d2*sin(s3) + m2 * l1*d2*sin(s3)) + v1 * v1*m2*l1*d2*sin(s3) + m2 * g*d2*sin(s2 + s3);
		T0 = goal_ang_acc_[0] * I0;
	} else if (robot_id_ == "JY-C" || robot_id_ == "JY-P") {
		double d1 = 0.028;double d2 = 0.139;double m1 = 5.5;double m2 = 0.645;double I1 = 0.06366;double I1_comp = 0.1;
		double I2 = 0.02494;double I2_comp = 0.12;double g = 9.8;double I0 = 0.1435; double I0_comp = 0.1;
		T1 = goal_ang_acc_[1] * (I1 + I2 + m2 * l1*l1 + 2 * m2*l1*d2*cos(s3) + I1_comp) + goal_ang_acc_[2] * (I2 + m2 * l1*d2*cos(s3))
			- 2 * m2*l1*d2*v1*v2 - m2 * l1*d2*v2*v2*sin(s3) + m1 * g*d1*sin(s2) + m2 * g*l1*sin(s2) + m2 * g*d2*sin(s2 + s3);
		T2 = goal_ang_acc_[1] * (I2 + m2 * l1*d2*cos(s3)) + goal_ang_acc_[2] * (I2 + I2_comp)
			+ v1 * v2*(m2*l1*d2*sin(s3) + m2 * l1*d2*sin(s3)) + v1 * v1*m2*l1*d2*sin(s3) + m2 * g*d2*sin(s2 + s3);
		T0 = goal_ang_acc_[0] * (I0 + I0_comp);
	} else if (robot_id_ == "JY-L") {
		double d1 = 0.0569;double d2 = 0.152;double m1 = 3.052;double m2 = 0.517;double I1 = 0.04629;double I1_comp = 0.05;
		double I2 = 0.02216;double I2_comp = 0.06;double g = 9.8;double I0 = 0.1965; double I0_comp = 0.1;
		T1 = goal_ang_acc_[1] * (I1 + I2 + m2 * l1*l1 + 2 * m2*l1*d2*cos(s3) + I1_comp) + goal_ang_acc_[2] * (I2 + m2 * l1*d2*cos(s3))
			- 2 * m2*l1*d2*v1*v2 - m2 * l1*d2*v2*v2*sin(s3) + m1 * g*d1*sin(s2) + m2 * g*l1*sin(s2) + m2 * g*d2*sin(s2 + s3);
		T2 = goal_ang_acc_[1] * (I2 + m2 * l1*d2*cos(s3)) + goal_ang_acc_[2] * (I2 + I2_comp)
			+ v1 * v2*(m2*l1*d2*sin(s3) + m2 * l1*d2*sin(s3)) + v1 * v1*m2*l1*d2*sin(s3) + m2 * g*d2*sin(s2 + s3);
		T0 = goal_ang_acc_[0] * (I0 + I0_comp);
	} else if (robot_id_ == "JY-M") {
		double d1 = 0.07;double d2 = 0.135;double m1 = 1.536;double m2 = 0.198;double I1 = 0.013;double I1_comp = 0.01;
		double I2 = 0.0061;double I2_comp = 0.01;double g = 9.8;double I0 = 0.029; double I0_comp = 0.01;
		T1 = goal_ang_acc_[1] * (I1 + I2 + m2 * l1*l1 + 2 * m2*l1*d2*cos(s3) + I1_comp) + goal_ang_acc_[2] * (I2 + m2 * l1*d2*cos(s3))
			- 2 * m2*l1*d2*v1*v2 - m2 * l1*d2*v2*v2*sin(s3) + m1 * g*d1*sin(s2) + m2 * g*l1*sin(s2) + m2 * g*d2*sin(s2 + s3);
		T2 = goal_ang_acc_[1] * (I2 + m2 * l1*d2*cos(s3)) + goal_ang_acc_[2] * (I2 + I2_comp)
			+ v1 * v2*(m2*l1*d2*sin(s3) + m2 * l1*d2*sin(s3)) + v1 * v1*m2*l1*d2*sin(s3) + m2 * g*d2*sin(s2 + s3);
		T0 = goal_ang_acc_[0] * (I0 + I0_comp);
	} else if (robot_id_ == "JY-S") {//TODO
		double d1 = 0.02451;double d2 = 0.07896;double m1 = 0.595;double m2 = 0.106;double I1 = 0.00186;double I1_comp = 0.005;
		double I2 = 0.001337;double I2_comp = 0.00625;double g = 9.8;double I0 = 0.000263; double I0_comp = 0.02;
		T1 = goal_ang_acc_[1] * (I1 + I2 + m2 * l1*l1 + 2 * m2*l1*d2*cos(s3) + I1_comp) + goal_ang_acc_[2] * (I2 + m2 * l1*d2*cos(s3))
			- 2 * m2*l1*d2*v1*v2 - m2 * l1*d2*v2*v2*sin(s3) + m1 * g*d1*sin(s2) + m2 * g*l1*sin(s2) + m2 * g*d2*sin(s2 + s3);
		T2 = goal_ang_acc_[1] * (I2 + m2 * l1*d2*cos(s3)) + goal_ang_acc_[2] * (I2 + I2_comp)
			+ v1 * v2*(m2*l1*d2*sin(s3) + m2 * l1*d2*sin(s3)) + v1 * v1*m2*l1*d2*sin(s3) + m2 * g*d2*sin(s2 + s3);
		T0 = 0*goal_ang_acc_[0] * (I0 + I0_comp);
	}
}

void LegControl::PelmaCompensation(double &s3_compensation, double &l2_compensation) {//consider the R of pelma
	double l1 = link_.link_length[1];
	double l2 = link_.link_length[2];
	double s2 = angle_[1];
	double s3 = angle_[2];
	double R = 0.03;//pelma R
	double alpha, beta;
	alpha = kPI - (s2 + s3);
	l2_compensation = sqrt(l2*l2 + R*R - 2 * l2*R*cos(alpha));
	beta = acos((l2*l2 + l2_compensation*l2_compensation - R*R) / (2 * l2*l2_compensation));
	s3_compensation = s3 - beta;
}

void LegControl::CoulombsCoefficient(Vec3 &coulombs, Vec3 goal_ang_vel_) {
	if (goal_ang_vel_[0]>0.005) {
		coulombs[0] = 1;
	}
	else if (goal_ang_vel_[0]<-0.005) {
		coulombs[0] = -1;
	}
	else {
		coulombs[0] = 200 * goal_ang_vel_[0];
	}

	if (goal_ang_vel_[1]>0.005) {
		coulombs[1] = 1;
	}
	else if (goal_ang_vel_[1]<-0.005) {
		coulombs[1] = -1;
	}
	else {
		coulombs[1] = 200 * goal_ang_vel_[1];
	}

	if (goal_ang_vel_[2]>0.005) {
		coulombs[2] = 1;
	}
	else if (goal_ang_vel_[2]<-0.005) {
		coulombs[2] = -1;
	}
	else {
		coulombs[2] = 200 * goal_ang_vel_[2];
	}
}

void LegControl::GetAbsMin(double &own, double ref, string var_name)
{
	if (abs(own) > abs(ref)) {
		own = Sign(own) * abs(ref);
		for (int i = 0; i < 10; i++) {
			if (global_.limit_var[i] == var_name) {
				break;
			}
			else if (global_.limit_var[i] == "") {
				global_.limit_var[i] = var_name;
				global_.limit_var_value[i] = Sign(own) * abs(ref);
				break;
			}
		}
	}
}

Vec6 LegControl::GetLegPositionGlobal()
{
	Vec3 calculate_xyz, calculate_xyz_vel;
	Vec6 hip_states;
	ForwardKinematic(angle_, velocity_, &calculate_xyz, &calculate_xyz_vel);
	calculate_xyz = -calculate_xyz;
	calculate_xyz_vel = -calculate_xyz_vel;
	calculate_xyz_vel = RolPitYawToRM(gyro_data_, 0) * calculate_xyz_vel + DerivRolPitYawToRM(gyro_data_, 0) * calculate_xyz;
	calculate_xyz = RolPitYawToRM(gyro_data_, 0) * calculate_xyz;
	hip_states << calculate_xyz[0], calculate_xyz[1], calculate_xyz[2], calculate_xyz_vel[0], calculate_xyz_vel[1], calculate_xyz_vel[2];
	return hip_states;
}

Vec3 LegControl::GroundContactDetection()
{
	double d1 = control_parameter_.d1; double d2 = control_parameter_.d2;
	double m1 = control_parameter_.m1; double m2 = control_parameter_.m2;
	double I0 = control_parameter_.I0; double I0_comp = control_parameter_.I0_comp;
	double I1 = control_parameter_.I1; double I1_comp = control_parameter_.I1_comp;
	double I2 = control_parameter_.I2; double I2_comp = control_parameter_.I2_comp;

	double l0 = link_.link_length[0];  //distance between joints
	double l1 = link_.link_length[1];
	double l2 = link_.link_length[2];
	double s0 = angle_[0];  // joint angle
	double s1 = angle_[1];
	double s2 = angle_[2];
	double v0 = velocity_[0];  // joint angle velocity (current cycle time)
	double v1 = velocity_[1];
	double v2 = velocity_[2];
	double last_v0 = global_.last_v[0];  // joint angle velocity (last cycle time)
	double last_v1 = global_.last_v[1];
	double last_v2 = global_.last_v[2];

	global_.current_velocity[0] = v0;
	global_.current_velocity[1] = v1;
	global_.current_velocity[2] = v2;

	//global_->last_v[0] = last_v0;
	//global_->last_v[1] = last_v1;
	//global_->last_v[2] = last_v2;

	global_.ang_acc[0] = (v0 - last_v0) / cycle_time_;
	global_.ang_acc[1] = (v1 - last_v1) / cycle_time_;
	global_.ang_acc[2] = (v2 - last_v2) / cycle_time_;

	global_.last_v[0] = velocity_[0];  //update joint angle velocity (last cycle time)
	global_.last_v[1] = velocity_[1];
	global_.last_v[2] = velocity_[2];

	Vec3 ang_acc; ang_acc[0] = 0.5*(v0 - last_v0) / cycle_time_; ang_acc[1] = 0.5*(v1 - last_v1) / cycle_time_; ang_acc[2] = 0.5*(v2 - last_v2) / cycle_time_;    //joint angle acceleration
	Vec3 pseudo_torque; Vec3 pseudo_torque_with_acc;
	Vec3 actual_torque; //Vec3 actual_torque_with_acc;
	Vec3 delta_torque;  Vec3 delta_torque_with_acc;
	Vec3 estimated_force_tmp; Vec3 estimated_force_tmp_with_acc;
	Vec3 estimated_force; Vec3 estimated_force_with_acc;

	// the following terms with acc data not change to JYB yet, because of not using them
	pseudo_torque[0] = 0;
	pseudo_torque_with_acc[0] = ang_acc[0] * (I0 + I0_comp);

	pseudo_torque[1] = -2 * m2*l1*d2*v1*v2 - m2 * l1*d2*v2*v2*sin(s2)
		+ m1 * kGravity*d1*sin(s1) + m2 * kGravity*l1*sin(s1) + m2 * kGravity*d2*sin(s1 + s2);
	pseudo_torque_with_acc[1] = ang_acc[1] * (I1 + I2 + m2 * l1*l1 + 2 * m2*l1*d2*cos(s2) + I1_comp)
		+ ang_acc[2] * (I2 + m2 * l1*d2*cos(s2))
		- 2 * m2*l1*d2*v1*v2 - m2 * l1*d2*v2*v2*sin(s2)
		+ m1 * kGravity*d1*sin(s1) + m2 * kGravity*l1*sin(s1) + m2 * kGravity*d2*sin(s1 + s2);

	pseudo_torque[2] = v1 * v2*(m2*l1*d2*sin(s2) + m2 * l1*d2*sin(s2)) + v1 * v1*m2*l1*d2*sin(s2)
		+ m2 * kGravity*d2*sin(s1 + s2);
	pseudo_torque_with_acc[2] = ang_acc[1] * (I2 + m2 * l1*d2*cos(s2))
		+ ang_acc[2] * (I2 + I2_comp)
		+ v1 * v2*(m2*l1*d2*sin(s2) + m2 * l1*d2*sin(s2)) + v1 * v1*m2*l1*d2*sin(s2)
		+ m2 * kGravity*d2*sin(s1 + s2);

	// transform from electric current to actual torque value
	//actual_torque[0] = (3.145 *pow(10, -5)*pow((real_torque_[0] / 1000 * 15) , 3) - 3.44 * pow(10, -18)*pow((real_torque_[0] / 1000 * 15) , 2) + 0.5812*(real_torque_[0] / 1000 * 15) + 9.41 * pow(10 , -16))*6.75 * 2;
	//actual_torque[1] = (3.145 *pow(10, -5)*pow((real_torque_[1] / 50) , 3) - 3.44 * pow(10, -18)*pow((real_torque_[1] / 50) , 2) + 0.5812*(real_torque_[1] / 50) + 9.41 * pow(10 , -16))*5.83;
	//actual_torque[2] = (3.145 *pow(10, -5)*pow((real_torque_[2] / 50), 3) - 3.44 * pow(10, -18)*pow((real_torque_[2] / 50) , 2) + 0.5812*(real_torque_[2] / 50) + 9.41 * pow(10 , -16))*5.83;

	actual_torque[0] = real_torque_[0];
	actual_torque[1] = real_torque_[1];
	actual_torque[2] = real_torque_[2];

	delta_torque[0] = actual_torque[0] - pseudo_torque[0];
	delta_torque[1] = actual_torque[1] - pseudo_torque[1];
	delta_torque[2] = actual_torque[2] - pseudo_torque[2];

	delta_torque_with_acc[0] = actual_torque[0] - pseudo_torque_with_acc[0];
	delta_torque_with_acc[1] = actual_torque[1] - pseudo_torque_with_acc[1];
	delta_torque_with_acc[2] = actual_torque[2] - pseudo_torque_with_acc[2];

	estimated_force_tmp[0] = ((l2*sin(s2 + s1) + l1 * sin(s1))*delta_torque[2] - l2 * sin(s2 + s1)*delta_torque[1]) / (l2*sin(s2 + s1)*l1*cos(s1) - l2 * cos(s1 + s2)*l1*sin(s1) + 0.00000001);
	estimated_force_tmp[2] = (l2*cos(s2 + s1)*delta_torque[1] - (l2*cos(s2 + s1) + l1 * cos(s1))*delta_torque[2]) / (l2*sin(s2 + s1)*l1*cos(s1) - l2 * cos(s1 + s2)*l1*sin(s1) + 0.00000001);
	estimated_force_tmp[1] = (estimated_force_tmp[2] * l0 + delta_torque[0]) / (l1*cos(s1) + l2 * cos(s1 + s2));

	estimated_force_tmp_with_acc[0] = ((l2*sin(s2 + s1) + l1 * sin(s1))*delta_torque_with_acc[2] - l2 * sin(s2 + s1)*delta_torque_with_acc[1]) / (l2*sin(s2 + s1)*l1*cos(s1) - l2 * cos(s1 + s2)*l1*sin(s1) + 0.00000001);
	estimated_force_tmp_with_acc[2] = (l2*cos(s2 + s1)*delta_torque_with_acc[1] - (l2*cos(s2 + s1) + l1 * cos(s1))*delta_torque_with_acc[2]) / (l2*sin(s2 + s1)*l1*cos(s1) - l2 * cos(s1 + s2)*l1*sin(s1) + 0.00000001);
	estimated_force_tmp_with_acc[1] = (estimated_force_tmp_with_acc[2] * l0 + delta_torque_with_acc[0]) / (l1*cos(s1) + l2 * cos(s1 + s2));
	Vec3 tmp;
	Vec3 body_rpy;
	Vec33 f_trans;
	body_rpy << gyro_data_.roll, gyro_data_.pitch, 0.0;
	tmp << -s0, 0, 0;
	f_trans = RolPitYawToRM(tmp).transpose() * RolPitYawToRM(body_rpy, 0).transpose();  //from this file line 607

	estimated_force = f_trans.inverse()* estimated_force_tmp;
	estimated_force_with_acc = f_trans.inverse()* estimated_force_tmp_with_acc;
	//P = 1 / (1 + exp(5.6949 - 0.043*estimated_force[2]));  //  parameters are trained with data from JYB2(2018.12.04)
	//global_->mat_trans = f_trans;
	//global_->mat_trans_inv = f_trans.inverse();

	global_.estimated_force_with_acc = estimated_force_with_acc;
	global_.ang_acc = ang_acc;
	return estimated_force;
}

void LegControl::VirtualForceToTorqueOutput(Vec3 force_xyz_global, Vec3 goal_angle, Vec3 goal_angle_vel)
{
	double l0 = link_.link_length[0];
	double l1 = link_.link_length[1];
	double l2 = link_.link_length[2];
	double s1 = angle_[0];
	double s2 = angle_[1];
	double s3 = angle_[2];

	double tor_hipx, tor_hipy, tor_knee;
	double force_x_local, force_y_local, force_z_local;
	double zt;
	double force_x_global_tmp = force_xyz_global[0];
	double force_y_global_tmp = force_xyz_global[1];

	Vec3 body_rpy;
	Vec3 force_transform;

	// prevent the leg get off the ground.
	if (force_xyz_global[2] < kMinFz) {
		force_xyz_global[2] = kMinFz;
	}

	GetAbsMin(force_x_global_tmp, kFrictionalCoefficientTrot * force_xyz_global[2], "virtual_force_x_global");
	GetAbsMin(force_y_global_tmp, kFrictionalCoefficientTrot * force_xyz_global[2], "virtual_force_y_global");

	GetAbsMin(force_x_global_tmp, kMaxFx, "virtual_force_x_global");
	GetAbsMin(force_y_global_tmp, kMaxFy, "virtual_force_y_global");

	force_xyz_global[0] = force_x_global_tmp;
	force_xyz_global[1] = force_y_global_tmp;

	zt = -(l1 * cos(s2) + l2 * cos(s2 + s3));
	//	s1t = s1 - asin(l0/zt);

	//Body_RPY << Gyro->Rol, Gyro->Pit, Gyro->Yaw;
	body_rpy << gyro_data_.roll, gyro_data_.pitch, 0.0;
	force_transform = VirtualForceCoordinateTransformFrom(force_xyz_global, body_rpy, s1);
	force_x_local = force_transform[0]; force_y_local = force_transform[1]; force_z_local = force_transform[2];
	virtual_force_local_[0] = force_x_local;
	virtual_force_local_[1] = force_y_local;
	virtual_force_local_[2] = force_z_local;

	tor_knee = -force_x_local * l2 * cos(s2 + s3) - force_z_local * l2 * sin(s2 + s3);
	tor_hipy = tor_knee - force_z_local * l1 * sin(s2) - force_x_local * l1 * cos(s2);
	tor_hipx = -force_y_local * zt - force_z_local * l0;

	double kp = 18; double kd = 0.9;
	torque_output_[0] = tor_hipx + kp*(goal_angle[0] - angle_[0]) + kd*(goal_angle_vel[0] - velocity_[0]);
	torque_output_[1] = tor_hipy + kp*(goal_angle[1] - angle_[1]) + kd*(goal_angle_vel[1] - velocity_[1]);
	torque_output_[2] = tor_knee + kp*(goal_angle[2] - angle_[2]) + kd*(goal_angle_vel[2] - velocity_[2]);
}
void LegControl::TorqueOutputDirectly(double tor_hipx, double tor_hipy, double tor_knee){
	torque_output_[0] = tor_hipx ;//+ kp*(goal_angle[0] - angle_[0]) + kd*(goal_angle_vel[0] - velocity_[0]);
	torque_output_[1] = tor_hipy ;//+ kp*(goal_angle[1] - angle_[1]) + kd*(goal_angle_vel[1] - velocity_[1]);
	torque_output_[2] = tor_knee ;//+ kp*(goal_angle[2] - angle_[2]) + kd*(goal_angle_vel[2] - velocity_[2]);
 // test 
   
	for(int i = 0; i < 12; i++){
		download_parameters.kp[i]=0;
		download_parameters.kd[i]=0;
		//download_parameters.t_ff[i]=0;
	}
if(leg_id == 0){
//cout<<"fl!!!!!!!!!!!"<<endl;
    download_parameters.t_ff[0] = tor_hipx;
    download_parameters.t_ff[1] = tor_hipy;
    download_parameters.t_ff[2] = tor_knee;}
if(leg_id == 1){
    download_parameters.t_ff[3] = tor_hipx;
    download_parameters.t_ff[4] = tor_hipy;
    download_parameters.t_ff[5] = tor_knee;}
if(leg_id == 2){
    download_parameters.t_ff[6] = tor_hipx;
    download_parameters.t_ff[7] = tor_hipy;
    download_parameters.t_ff[8] = tor_knee;}
if(leg_id == 3){
    download_parameters.t_ff[9] = tor_hipx;
    download_parameters.t_ff[10] = tor_hipy;
    download_parameters.t_ff[11] = tor_knee;}
   // cout<<"t_ff[0] = "<<download_parameters.t_ff[0]<<endl; 
   // cout<<"t_ff[1] = "<<download_parameters.t_ff[1]<<endl;   
  //  cout<<"t_ff[2] = "<<download_parameters.t_ff[2]<<endl;     
 //
}
void LegControl::TorqueOutputDirectly(Vec3 tor,Vec3 ang,Vec3 vel){
	torque_output_[0] = -tor[0] ;//+ kp*(goal_angle[0] - angle_[0]) + kd*(goal_angle_vel[0] - velocity_[0]);
	torque_output_[1] = tor[1] ;//+ kp*(goal_angle[1] - angle_[1]) + kd*(goal_angle_vel[1] - velocity_[1]);
	torque_output_[2] = tor[2] ;//+ kp*(goal_angle[2] - angle_[2]) + kd*(goal_angle_vel[2] - velocity_[2]);

	for(int i = 0; i < 12; i++){
		download_parameters.kp[i]=0;
		download_parameters.kd[i]=0;
		//download_parameters.t_ff[i]=0;
	}
   for (int leg = 0; leg < 4; leg++) {
    if(leg_id == leg){

   download_parameters.goal_angle[3*leg+0] = ang[0];
   download_parameters.goal_angle_vel[3*leg+0] = vel[0];
   download_parameters.t_ff[3*leg+0] = -tor[0];
   
   download_parameters.goal_angle[3*leg+1] = ang[1];
   download_parameters.goal_angle_vel[3*leg+1] = vel[1];
   download_parameters.t_ff[3*leg+1] = tor[1];
   
   download_parameters.goal_angle[3*leg+2] = ang[2];
   download_parameters.goal_angle_vel[3*leg+2] = vel[2];
   download_parameters.t_ff[3*leg+2] = tor[2];
   }
  }

   
}
