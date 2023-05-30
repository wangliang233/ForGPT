
#include "version_ras.h"
#include "inner_type.h"

#ifndef PARAMETERS_SHARE_H_
#define PARAMETERS_SHARE_H_

#define PROGRAM_RUN  // program state. {PROGRAM_TEST, PROGRAM_RUN}

const double kGravity = 9.8;
const int kInitialWorkingTemperature = 70;  // less than 70 degrees centigrade
const int kWarningWorkingTemperature = 100; //greater than 90 degrees centigrade, the robot begin warning
const int kMaxWorkingTemperature = 115;  // greater than 90 degrees centigrade, the robot stop action

const double kSlamLocationStartTheta = 1.56184;
const double kSlamLocationStartX = 8.47926;
const double kSlamLocationStartY = 3.77567;

const double kSlamLocationEndTheta = 1.5498;
const double kSlamLocationEndX = 8.49513;
const double kSlamLocationEndY = 4.27733;

const double kChargeTargetTheta = -0.2549 * kDegree2Radian;//0.00706* kDegree2Radian;//-0.1293 * kDegree2Radian;//-0.53195 * kDegree2Radian;
const double kChargeTargetLocationX = 1.30986;//1.31058;//1.32806;//1.33258;
const double kChargeTargetLocationY = 0.000428;//0.00849;//-0.0104;//-0.00190557;

// **********  Velocity Control Parameters  **********
const double kTurningVelocityMax = 50 * kDegree2Radian;
const double kForwardVelocityLow = 0.6;
const double kForwardVelocityHigh = 2.5;
const double kBackwardVelocityHigh = 1.3;
const double kForwardVelocityHighJYM = 1.8;
const double kBackwardVelocityHighJYM = 1.2;
const double kForwardVelocityHighJYS = 3.6;
const double kBackwardVelocityHighJYS = 1.8;
const double kSideVelocityMax = 0.5;
const double kRightVelocityLow = -0.3;
const double kLeftVelocityLow = 0.3;
// **********  Velocity Control Parameters  **********

const double kYawVelocityMax = 20*kDegree2Radian;
const double kYawMax = 30*kDegree2Radian;
const double kPitchVelcocityMax = 20*kDegree2Radian;
const double kPitchMax = 20*kDegree2Radian;
// pitch property is determined by body height
const double kRollVelocityMax = 10*kDegree2Radian;
const double kRollMax = 30*kDegree2Radian;
const double kZVelocityMax = 0.02;
//z property is determined by pitch and robot

// **********  Virtual Force Control Parameters  **********
const double kMaxFx = 300;
const double kMaxFy = 300;
const double kMinFz = 0;
const double kFrictionalCoefficientTrot = 0.65;//0.3;
// **********  Virtual Force Control Parameters  **********



const HardwareParameter kHardwareParameterJYS = {
	{ -0.0875, 0.18, 0.19 },	// FL
	{ 0.0875,  0.18, 0.19 },	// FR
	{ -0.0875, 0.18, 0.19 },	// HL
	{ 0.0875,  0.18, 0.19 },	// HR
	{//TODO
		0.178,		//Lmx, dis between f & h hip axis
		0.055,		//Lmy, dis between l & r hip axis
		-0.0,		//Lmz
		0.12,
	  10.5//8.7		//M
	}
};

const ControlParameter kDefaultControlParameterJYS = {
	// ***** Leg *****
	{ 60,   // hipx_kp//700
	0.7,   // hipx_kd//2.4
	90,  // hipy_kp
	0.7,    // hipy_kd
	80,  // knee_kp//650//580//1000
	1.2,    // knee_kd//15
	{ 20, 30, 30 },// tor_lim
	1.5,  0.025*1, 0, 0,  // swing leg compensation
	0.07,   0.135,  // d1, d2
	1.536,  0.198,	// m1, m2
	0.029,  0.01,	// I0, I0_comp
	0.013,  0.01,  // I1, I1_comp
	0.0061, 0.01 }, // I2, I2_comp

				// ***** Body Posture Trot *****
	{ 0,   // fx_kp
	20, // fx_kd
	0, // fy_kp
	0,   // fy_kd
	3500, // fz_kp//0.7
	25,   // fz_kd//0.8
	18,   // roll_kp   // 500//0.45
	1,    // roll_kd   // 20//0.8
	150,  // pitch_kp  // 1000//0.6
	10,    // pitch_kd  // 50//0.8
	105,  // yaw_kp//0.5
	6,   // yaw_kd//0.6
	true
	},
	{
						380,   // fx_kp
						20, // fx_kd
						350, // fy_kp
						20,   // fy_kd
						4500 , // fz_kp//0.7
						60 ,
						80,   // roll_kp   // 500//0.45
						2,    // roll_kd   // 20//0.8
						200,  // pitch_kp  // 1000//0.6
						8,    // pitch_kd  // 50//0.8
						100,  // yaw_kp//0.5
						6,   // yaw_kd//0.6
						true
	}
};

#endif  //PARAMETERS_SHARE_H_
