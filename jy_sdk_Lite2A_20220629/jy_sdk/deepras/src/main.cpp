
#include "controller/motion.h"
#include "motion/basic_motion.h"
#include "version_ras.h"
extern TaskState task_state = {0};
extern MotionConfig inner_motion_config = {0};
extern RobotActionState robot_action_state = {0};
extern RobotStateUpload robot_state_upload = {0};
extern DownLoadParameters download_parameters = {0};
static int control_state_record = kBasicMotion;
static RobotState actual_robot_state = {0};
static RobotState goal_robot_state = {0};
extern RobotJointAngle robot_joint_angle = {0};
extern RobotJointVel robot_joint_vel = {0}; 
extern RobotActionUpload robot_action_upload = {0}; 
extern RobotCmdUpload robot_cmd_upload = {0};  
static LegControl fl(0), fr(1), hl(2), hr(3);
static basic_motion::BasicMotion robot_basic_motion(&fl, &fr, &hl, &hr, &goal_robot_state, &actual_robot_state);

string ShowVersion()
{
		return DEEPRAS_VERSION_NAME;
}

void ScopeNamesOutput()
{
	basicmethod::ScopeNamesOutput();
}

void Plan(){
}

uint32_t PlanPeroid() {
	return 30;
}

void MotionInitial(const string &robot_id, const string &robot_type, const MotionConfig &external_motion_config)
{
	MotionConfigInitial(robot_id, robot_type, external_motion_config);
	fl.ConstParameterInitial(robot_id, "FL");
	fr.ConstParameterInitial(robot_id, "FR");
	hl.ConstParameterInitial(robot_id, "HL");
	hr.ConstParameterInitial(robot_id, "HR");
	robot_basic_motion.ConstParameterInitial(robot_id, robot_type);
}

void Motion(const DataLegs *leg_input, const DataGyro &gyro_data_raw, const double &time_stamp, const double &cycle_time, const ControlCommands &control_commands_raw,
			scope_output *scope_output, DataLegs *leg_output)
{
	DataGyro gyro_data_process;
	InnerControlCommands inner_control_commands = {0};
	DataPreProcessing(robot_basic_motion.robot_id_, control_state_record, actual_robot_state, gyro_data_raw, control_commands_raw, time_stamp, &gyro_data_process, &inner_control_commands, scope_output, leg_output);
//	IsRobotLoseControl(actual_robot_state,goal_robot_state,gyro_data_process);

	fl.DataInputProcess(leg_input->fl_pos, leg_input->fl_vel, leg_input->fl_torque, gyro_data_process, cycle_time);
	fr.DataInputProcess(leg_input->fr_pos, leg_input->fr_vel, leg_input->fr_torque, gyro_data_process, cycle_time);
	hl.DataInputProcess(leg_input->hl_pos, leg_input->hl_vel, leg_input->hl_torque, gyro_data_process, cycle_time);
	hr.DataInputProcess(leg_input->hr_pos, leg_input->hr_vel, leg_input->hr_torque, gyro_data_process, cycle_time);

	switch (control_state_record)
	{
		case kBasicMotion: {
			robot_basic_motion.DataInputProcess(inner_control_commands.basic, time_stamp, cycle_time, gyro_data_process);
			robot_basic_motion.MotionProcess();
			break;
		}
	}
	RobotStateOutput(goal_robot_state, actual_robot_state);
	OutputDataProcessing(fl.goal_angle_, fl.goal_ang_vel_, fl.torque_output_, fr.goal_angle_, fr.goal_ang_vel_, fr.torque_output_, 
						 hl.goal_angle_, hl.goal_ang_vel_, hl.torque_output_, hr.goal_angle_, hr.goal_ang_vel_, hr.torque_output_, fl.control_parameter_, leg_output);
}
