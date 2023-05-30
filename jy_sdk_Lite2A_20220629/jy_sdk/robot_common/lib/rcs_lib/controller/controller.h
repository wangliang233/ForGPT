/*
 * controller.h
 *
 *  Created on: 2018-3-21
 *      Author: moxiaobo
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "motion.h"
#include "control_profile.h"

#include "utility/base_actor.h"
#include "motor/joint_data.h"
#include "service/ServiceCharger.h"
#include "service/ServiceChargerV2.h"

#include "robot_picklist.h"
#include <signal.h>

const size_t kHeatChanelSize = 16;
struct HeatData{
	float timestamp;
	int16_t buffer[kHeatChanelSize];
};

typedef int UpHeatData[12];

namespace deepros {namespace controller{
  using namespace robot_profile::activated_robot;
  using system_types::BaseActor;
  using motor::JointData;
  const string kDeeprosVersion = "000103";

class Controller: public BaseActor {
public:
  static Controller* GetInstance(double algorithm_period,
                                 JointData* joint_data_raw,
                                 const system_types::ImuData* imu_data_raw,string robot_name){
    if(controller_ == 0)
      controller_ = new Controller(algorithm_period,joint_data_raw,imu_data_raw,robot_name);
    return controller_;
  }
  static void FreeInstance(){
    if(controller_ != 0) delete controller_;
  }
  virtual ~Controller();

  const JointMetricData& get_joint_metric_actual_data();
  const JointMetricData& get_joint_metric_target_data();
  uint8_t slave_order_[kSlaveSize];

  scope_output& get_scope_data(){
    return scope_data_;
  }
  size_t get_scope_data_size(){
    return sizeof(scope_data_)/sizeof(double);
  }
  void set_heat_data_raw(const system_types::HeatData & heat_data){
  	heat_data_raw_ = &heat_data;
  }
  // void set_driver_temp_data_raw(DriveTemperature &driver_temp_data){
  //   // for(int i=0;i<12;i++){
  //   //   driver_temp_data_raw[i] = driver_temp_data[i];
	// 	//   printf("  ,%f\n",driver_temp_data[i]/1e2);
  //   // }

  //    driver_temp_data_raw = driver_temp_data;
  // }
  void install_segfault_handler();
  void StrToDouble(const char* target_velocity, const int size);
  virtual void KineticsDeduce(double&timestamp);

  bool VerifyCompatibility(const string &deepros, const string &target_robot, const string &target_system);
  bool ReadRobotConfigiration();

  void SetDriveWanringHeat(bool flag = true);

  void InitialKinetics(const string &robot_id,const string &robot_type){
#ifndef OUTSIDER
    motion_config_.forward_velocity_offset.valid = manager_->main_config_.motion_config_.forward_velocity_offset.valid;
    motion_config_.forward_velocity_offset.value = manager_->main_config_.motion_config_.forward_velocity_offset.value;

    motion_config_.side_velocity_offset   .valid = manager_->main_config_.motion_config_.side_velocity_offset   .valid;
    motion_config_.side_velocity_offset   .value = manager_->main_config_.motion_config_.side_velocity_offset   .value;

    motion_config_.stand_height           .valid = manager_->main_config_.motion_config_.stand_height           .valid;
    motion_config_.stand_height           .value = manager_->main_config_.motion_config_.stand_height           .value;
    motion_config_.backflip_hip_kp[0] = manager_->main_config_.GetConfigContext().get("backflip_hip_kp_fl","").asDouble();
    motion_config_.backflip_hip_kp[1] = manager_->main_config_.GetConfigContext().get("backflip_hip_kp_fr","").asDouble();
    motion_config_.backflip_hip_kp[2] = manager_->main_config_.GetConfigContext().get("backflip_hip_kp_hl","").asDouble();
    motion_config_.backflip_hip_kp[3] = manager_->main_config_.GetConfigContext().get("backflip_hip_kp_hr","").asDouble();
    motion_config_.backflip_knee_kp = manager_->main_config_.GetConfigContext().get("backflip_knee_kp","").asDouble();
    motion_config_.sideflip_abad_kp = manager_->main_config_.GetConfigContext().get("sideflip_abad_kp","").asDouble();
    motion_config_.sideflip_hip_kp = manager_->main_config_.GetConfigContext().get("sideflip_hip_kp","").asDouble();
    motion_config_.sideflip_knee_kp = manager_->main_config_.GetConfigContext().get("sideflip_knee_kp","").asDouble();
    MotionInitial(robot_id,robot_type,motion_config_);
#else
    MotionInitial(robot_id);
#endif

  }

  void set_charger(service::ServiceChargerV2 *charger){
    charger_ = charger;
  }

  deeprobotics::quadruped::Robot& activated_robot_;
  pthread_t GetThreadId(){
    return thread_id_;
  }
  uint32_t get_plan_peroid(){
    return plan_peroid_;
  }

  enum Signal{
    kMpcSignal = SIGUSR1
  };

protected:
  virtual void* RunRoutine(void*);
  static Controller *controller_;

  const system_types::ImuData* imu_data_raw_;
  const system_types::HeatData* heat_data_raw_;
 // DriveTemperature* driver_temp_data_raw;
  service::ServiceChargerV2 *charger_;
  DataGyro imu_data_;
  JointData* joint_data_pointer_;
  deeprobotics::quadruped::JointMetricData joint_metric_real_data_,joint_metric_actual_data_,joint_metric_target_data_;

  ControlCommands user_command_set_;
  scope_output scope_data_;
  double algorithm_period_;
  int pos_comp[12] = {0};

  string robot_serial_number_;
#ifndef OUTSIDER
  MotionConfig motion_config_;
#endif
  uint32_t plan_peroid_;
  void ParseCommand(deepros::command::Command &c,ControlCommands &user_command_set_);
  Controller(double algorithm_period,
      JointData* joint_data_raw,
      const system_types::ImuData* imu_data_raw,string robot_name)
    :BaseActor(5,SCHED_RR,80*1000*1000,3),
     activated_robot_(GetRobot(robot_name)),
     joint_data_pointer_(joint_data_raw){

    algorithm_period_ = algorithm_period;
    imu_data_raw_ = imu_data_raw;
    heat_data_raw_ = 0;
    charger_ = 0;

    std::memset(&scope_data_,0,sizeof(scope_data_));
    std::memset(&user_command_set_,0,sizeof(user_command_set_));
#ifndef OUTSIDER
    user_command_set_.gait = 3;
    std::memset(&motion_config_,0,sizeof(motion_config_));
#endif
    std::memset(&joint_metric_target_data_,0,sizeof(joint_metric_target_data_));


    uint8_t slave_order[kSlaveSize]={/*FLX,FLY,FLK, = */8,7,6,
																		 /*FRX,FRY,FRK, = */3,4,5,
																		 /*HLX,HLY,HLK, = */9,10,11,
																		 /*HRX,HRY,HRK, = */2,1,0};
    std::memcpy(slave_order_,slave_order,sizeof(slave_order));

    plan_peroid_ = PlanPeroid();
    if( plan_peroid_ != 0 && (plan_peroid_ < 10 || plan_peroid_ > 2e3)){
      plan_peroid_ = 0;
    }

  }


};

}}

#endif /* CONTROLLER_H_ */
