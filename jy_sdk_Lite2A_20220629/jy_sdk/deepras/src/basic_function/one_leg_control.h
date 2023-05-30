
#ifndef BASIC_ONE_LEG_CONTROL_H_
#define BASIC_ONE_LEG_CONTROL_H_

#include "basic_method.h"

using namespace basicmethod;

// LegControl control one leg to complete some action, including position model control and virtual force control model.
// Also including the FK and IK of the legs, and output process, such as output limitation, real torque transform and so on.
class LegControl {
 public:
  LegControl(int i);
  ~LegControl();

  void ConstParameterInitial(const string &robot_id, const string &leg);
  void DataInputProcess(const OneTypeLegData &leg_pos_input, const OneTypeLegData &leg_vel_input, const OneTypeLegData &leg_torque_input,
						const DataGyro &gyro_data_process, const double &cycle_time);

  Vec3 CalculateAngleGlobal(Vec3 goal_xyz);
  Vec3 CalculateAngleLocal(Vec3 goal_xyz);
  Vec3 CalculateAngleConsiderPitch(Vec3 goal_xyz);
  Vec3 InverseKinematic(Vec3 goal_xyz);
  Vec3 InverseKinematicGlobal(Vec3 goal_xyz);
  Vec3 CalculateToeAngles();
  Vec6 CalculateHipStatesGlobal();
  Vec6 CalculateHipStatesLocal();
  Vec6 GetLegPositionGlobal();
  Vec66 CalculateJacobian();
  void SwingToPosition(int n);
  void SwingToPosition(Vec3 goal_pos, Vec3 goal_pos_next, Vec3 goal_pos_next2, int n, string side);
  void SwingToPosition(Vec3 goal_pos, Vec3 goal_pos_next, Vec3 goal_pos_next2, int n, string side, double kp_rate);
  void SwingToPosition(Vec3 goal_pos, Vec3 goal_pos_next, Vec3 goal_pos_next2, int n, string side, double kp_rate ,double kd_rate);
  void SwingToPositionAnglePlan(Vec3 initial_angle, Vec3 goal_xyz, double total_time, double run_time, string side, int i);
  void SwingToPositionHipY(double goal_angle, string side);
  void SwingToPosition(double *goal_pos, string side);
  void SwingToPositionStandUpAndDown(Vec3 goal_pos, Vec3 goal_pos_next, Vec3 goal_pos_next2, string side);
  void Initial();
  void ForwardKinematic(const Vec3 &angle, const Vec3 &ang_vel, Vec3 *calculate_xyz, Vec3 *calculate_xyz_vel);
  void ForwardKinematicLowPassFilter(Vec3 &angle, Vec3 &ang_vel, Vec3 &calculate_xyz, Vec3 &calculate_xyz_vel, string side);
  void ForwardKinematic(Vec3 *calculate_xyz, Vec3 *calculate_xyz_vel);
  void ForwardKinematicXYZ(Vec3 *calculate_xyz);
  void ForwardKinematicCoordinateTransform(Vec3 *calculate_xyz);
  void TorqueOutputLimit();
  void VirtualForceToTorqueOutput(Vec3 force_xyzF);
  void VirtualForceToTorqueOutput(Vec3 force_xyz, double Fz_min);
  void VirtualForceToTorqueOutputPronk(Vec3 force_xyz_global, double FricCoef);
  void CalculateActualVirtualForce(Vec3 &force_xyz);
  void DynamicsCompensation(Vec3 goal_angle_, Vec3 goal_angle_next, Vec3 goal_angle_next2, double &T1, double &T2, double &T0);
  void PelmaCompensation(double &s3_compensation, double &l2_compensation);
  void CoulombsCoefficient(Vec3 &coulombs, Vec3 goal_ang_vel_);
  void GetAbsMin(double &own, double ref, string var_name);
  Vec3 GroundContactDetection();

  void VirtualForceToTorqueOutput(Vec3 force_xyz_global, Vec3 goal_angle, Vec3 goal_angle_vel);
  void TorqueOutputDirectly(double tor_hipx, double tor_hipy, double tor_knee);
  //void TorqueOutputDirectly(Vec3 tor);
void TorqueOutputDirectly(Vec3 tor,Vec3 ang,Vec3 vel);
  OneLegAttribute link_;
  Vec3 angle_;
  Vec3 velocity_;
  Vec3 real_torque_;
  Vec3 goal_pos_;
  Vec3 goal_pos_next_;
  Vec3 goal_pos_next2_;
  Vec3 goal_pos_vel_;
  Vec3 goal_angle_;
  Vec3 goal_ang_vel_;
  Vec3 goal_ang_vel_next_;
  Vec3 goal_ang_acc_;
  Vec3 virtual_force_local_;
  OneLegControlParameter control_parameter_;
  DataGyro gyro_data_;
  double cycle_time_;
  static DataGlobalOneLeg global_;
  Vec3 toe_angles_;
  Vec66 jacobian_;
  Vec6 hip_states_global_;
  Vec6 hip_states_local_;
  double knee_vel_offset_;
  double knee_angle_offset_;
  double hipy_vel_offset_;
  double hipy_angle_offset_;
  Vec3 torque_output_;
  double vel_in[3];
  double vel_out[3];
  int leg_id;
  double kk0,kk1,kk2;

 private:
  const OneLegAttribute *leg_attribute_;
  string robot_id_;
};  // LegControl

#endif  // BASIC_ONE_LEG_CONTROL_H_
