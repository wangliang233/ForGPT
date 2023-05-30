/*
 * control_profile.h
 *
 *  Created on: 2018-4-4
 *      Author: moxiaobo
 */

#ifndef CONTROL_PROFILE_H_
#define CONTROL_PROFILE_H_
#include <stdint.h>

namespace deepros {namespace controller{

  namespace user_command{
    union UserCommand {
      uint32_t code;
      struct {
        uint32_t item  :8;
        uint32_t group :8;
        uint32_t type  :8;
        uint32_t source:8;
      };
      struct {
        uint32_t cmd :16;
        uint32_t src :16;
      };
    };
    enum Default{
      kDefault = 0x00,
    };
    enum CommandMask{
      kSourceMask   = 0xFF000000,
      kTypeMask     = 0x00FF0000,
      kGroupMask    = 0x0000FF00,
      kItemMask     = 0x000000FF,
    };

    enum CommandSource{
      kMotion       = 0x10000000,
      kGamePad      = 0x20000000,
      kSlam         = 0x30000000,
      kAI           = 0x40000000,
//      kVision       = 0x40000000,
//      kVoice        = 0x50000000,
//      kGesture      = 0x60000000,
      kServer       = 0x70000000,

    };

    enum CommandSourceVersion{
      kV1           = 0x01000000,
      kV2           = 0x02000000,
      kV3           = 0x03000000,
      kV4           = 0x04000000,
      kV5           = 0x05000000,
      kV6           = 0x06000000,
      kV7           = 0x07000000,
      kV8           = 0x08000000,
      kV9           = 0x09000000,
      kV10          = 0x0A000000,
      kV11          = 0x0B000000,
      kV12          = 0x0C000000,
      kV13          = 0x0D000000,
      kV14          = 0x0E000000,
      kV15          = 0x0F000000,
    };

    enum CommandType{
      kCommand      = 0x00010000,
      kRequest      = 0x00020000,
      kRespond      = 0x00030000,
      kStandby      = 0x00040000,  /// 旁路维护指令，例如心跳包0x0001
      kStatus       = 0x00050000,
      kBroadcast    = 0x00090000,

      kDebug        = 0x000A0000,
      kInfo         = 0x000B0000,
      kNotice       = 0x000C0000,
      kWarn         = 0x000D0000,
      kError        = 0x000E0000,
      kFault        = 0x000F0000,
    };

    enum MotionCommandGroup{
      kMotionAxis   = 0x0100,
      kStandPlay    = 0x0200,
      kGait         = 0x0300,
      kTerrain      = 0x0400,

      kManage       = 0x0C00,  /// 运动系统管理指令，例如保存退出0x01
      kSpecific     = 0x0D00,
      kService      = 0x0E00,
      kSafe         = 0x0F00,
    };

    // TODO
    enum MotionScopeGroup{
      kJoint        = 0x0000,
      kCentroid     = 0x0100,
      kStatistics   = 0x0200,
      kImu          = 0x0300,
      kRobot        = 0x0F00,
    };
    enum MotionScopeItem{
      kBattery      = 0x01,
      kMotorHeat    = 0x02,
      kDriverHeat   = 0x03,
      kControlPerf  = 0x04,
      kEthercatPerf = 0x05,
    };

    // kMotionAxis   = 0x0100
    enum MotionAxis{
      kX            = 0,
      kY            = 1,
      kZ            = 2,
      kRX           = 3,
      kRY           = 4,
      kRZ           = 5,
      kMaxX         = 0x10,
      kMaxY         = 0x11,
      kMaxZ         = 0x12,
      kMaxRX        = 0x13,
      kMaxRY        = 0x14,
      kMaxRZ        = 0x15,

      kSpeedX       = 0x30,
      kSpeedY       = 0x31,
      kSpeedZ       = 0x32,
      kSpeedRX      = 0x33,
      kSpeedRY      = 0x34,
      kSpeedRZ      = 0x35,
      kMaxSpeedX    = 0x20,
      kMaxSpeedY    = 0x21,
      kMaxSpeedZ    = 0x22,
      kMaxSpeedRX   = 0x23,
      kMaxSpeedRY   = 0x24,
      kMaxSpeedRZ   = 0x25,

    };

    // kStandPlay    = 0x0200
    enum StandPlay{
      kEnable       = 1,  /// 使能
      kStand        = 2,  /// 起立
      kSitDown      = 3,  /// 坐下
      kDance        = 4,  /// 跳舞
      kBackflip     = 5,  /// 后空翻
      kJumplong     = 6,  /// 跳远
      kJumpHigh     = 7,  /// 跳高
      kRollOver     = 8,  /// 翻滚
      kSquat        = 9,  /// 半蹲
      kJumpRot      = 10, /// 转身跳
      kGreeting1     = 11, /// 打招呼1
      kShot         = 12, /// 射门
      kSideflip     = 13, /// 侧翻
      kGreeting2    = 14, /// 打招呼2
      kGreeting3     = 15, /// 打招呼3
    };
    // kManage       = 0x0C00
    enum MotionManage{
      kSaveData     = 0x1,
      kManualControl= 0x2,
      kAutoControl  = 0x3,
      kNeedCharge   = 0x4,
      kNeedReset    = 0x5,  /// 关节回零
      kAutoStop     = 0x6,  /// 保持运动
      kQuickStop    = 0xD,
      kQuickSit     = 0xE,
      kQuickLock    = 0xF,
      kVoice        = 0xA,
    };
    // kGait         = 0x0300
    enum MotionGait{
      kTrot         = 0,
      kFlyTrot      = 1,
      kBound        = 2,
      kGallop       = 3,
      kPace         = 4,
      kWalk         = 5,
      kPronk        = 6,
      kTrotRun      = 7,
      kStairTrot    = 8,
      kDance2       = 9,
      kDance3       = 0XA,
      kDance4       = 0XB,
      kWalkingpg    = 0XC,
    };

    // kTerrain      = 0x0400
    enum MotionTerrain{
      kStandardTerrain = 0,
      kStairTerrain    = 1,
      kSlopeTerrain    = 2,
      kSlippedTerrain  = 3,
      kGravelTerrain   = 4,
      kRuggedTerrain   = 5,
      kLowHeightTerrain= 6,
    };

    // kService      = 0x0E00
    enum MotionService{
      kExactGo         = 0,
      kTakePhoto       = 1,
      kAimTarget       = 2,
    };

    // kSpecific     = 0x0D00
    enum MotionSpecific{
      kSlamCorrect     = 0,
      kVisionCorrect   = 1,
      kArmCorrect      = 2,
      kManualOperate   = 3,
      kAiOperate       = 4,  /// 切入导航或智能控制模式
      kAxisForStand    = 5,  /// 轴用于控制原地扭动身体
      kAxisForMotion   = 6,  /// 轴用于控制运动方向
    };

    /// 用户控制接口
    /// @Deprecated
    enum UserControlMap{
      kButtonOption 					= 1,  ///< button back
      kButtonStart 						= 2,  ///< button start
      kButtonY 								= 3,  ///< button Y
      kChangeAcceleration     = 4,
      kButtonA    						= 5,
      kScopeNamesOutputFlag   = 6,
      kButtonLB          			= 7,  ///< left button
      kButtonRB           		= 8,  ///< right button
      kRightAxisX             = 9,  ///< right x axis
      kRightAxisY         		= 109,  ///< right y axis
      kLeftAxisX              = 110, ///< left x axis
      kLeftAxisY              = 10, ///< left y axis
      kLTandRT         				= 11, ///< left and right triger
      kLidarWarningDistance     = 12,  // mm
      kTargetVelocity           = 14,
      kStopButton               = 15, ///< safe stop button
      kTouchButton              = 16,
      kVoiceCommand             = 17,  // 1: stand up  2: stand down
      kButtonBAndDigitalPad     = 18, ///< button B and digital pad
      kButtonX            		  = 19, ///< button X
      kLandMarkRotationPosition = 20,
      kWiFiConnectTimeRecord    = 21,
      kJoystickBreakLineFlag    = 22,
      kDpadUp					          = 23,
      kDpadDown					        = 24,
      kDpadLeft					        = 25,
      kDpadRight				        = 26,
      kGesture					        = 27,
      kRetryCharge              = 28,
      kChargeOver               = 29,
      kRobotSatae               = 30,
      kBatteryLow               = 31,
//      kVoiceInput               = 32,
      kSlamConnectTimeRecord    = 33,
      kGestureV2								= 32,

      kMuseumProject            = 50,
      kTargetPoint              = 51,
      kLidarCurrentPos          = 52,


      kDriverFatalError         = 0xC0000001,
      kSoftEmergency            = 0xF0000001,

      kAppStandUpStandDown         = 0x0101,  ///< 起立或坐下间切换，手机控制时，此命令包含2、3命令
      kAppSwitchToForce            = 0x0102,  ///< 保留——切换到力控制
      kAppStartStopAction          = 0x0103,  ///< 保留——开始或停止运动
      kAppControlModeSwitchDown    = 0x0107,  ///< 下一个运动模式
      kAppControlModeSwitchUp      = 0x0108,  ///< 上一个运动模式
      kAppHeadAngleControl         = 0x0109,  ///< 转向控制  （x-10）>0？右转：左转
      kAppForwardSpeedControl      = 0x010A, ///< 前进控制  （x-10）>0？前进：后退   有两级
      kAppSideSpeedControl         = 0x010B, ///< 左右平移  0 - 左移  1 - 右移
      kAppTargetVelocity           = 0x010E, ///< 目标速度
      kAppHeartBeat                = 0x010F, ///< 心跳
      kAppStepHeight               = 0x0110, ///< 台阶高度
      kAppStepWidth                = 0x0111, ///< 步间距
      kAppModeChange               = 0x0112, ///< 控制模式切换 0 - 手动  1 - 导航
      kAppDanceMode                = 0x0113, ///< 跳舞模式 0 - 跳舞  1 - 抬头 2 - 扭身体 3 - 招手 4 - 其他
      kAppStopConnect              = 0x0114, ///< 与机器人断开连接
      kAppSendAddress              = 0x0115, ///< 发送本机地址，使用ExtraCommand
      kAppQuickRoll                = 0x0116, ///< 快速翻滚

      kSlamYawCorrect           = 0x0121,
      kYawVel										= 0x0122,
      kXDirectionVel						= 0x0123,
      kYDirectionVel            = 0x0124,
      kYawAcc										= 0x0125,
      kXDirectionAcc						= 0x0126,
      kYDirectionAcc						= 0x0127,
      kFrictionState						= 0x0128,


      kVisionSlamVelX           = 0x0140,
      kVisionSlamYawVel         = 0x0141,

      kObstacleDistance         = 0x0142,
      kVisionSlamYawAngle       = 0x0143,
      kObstacleDistanceBack     = 0x0144,
      kVisionSlamVelY           = 0x0145,

      kVisionshot               = 0x01F7,
      kVisionJumplong           = 0x01F8,
      kVisionSquat              = 0x01F9,
      kVisionBackflip           = 0x01FA,
      kVisionStand              = 0x01FB,

      kGripMapPosture           = 600,
    };
  }
}}

#endif /* CONTROL_PROFILE_H_ */
