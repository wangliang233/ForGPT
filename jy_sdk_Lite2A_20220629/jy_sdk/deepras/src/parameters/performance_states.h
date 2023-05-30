
#ifndef PERFORMANCE_STATES_H_
#define PERFORMANCE_STATES_H_
namespace performance {
	enum TaskStateType{
		kDance         = 31,
		kJump          = 30,
		kNearToCharge  = 29,
		kSitToCharge   = 28,
		kStandUp       = 26,
		kStandDown     = 25,
		kForceControl  = 24,
		kLookUp        = 23,
		kLookDown      = 22,
		kQuickRoll     = 21,    ///< 软站立->滚动->软站立
		kTurnOver      = 20,    ///< 失控->趴
	};

	enum RobotActionStateType {
		kLie           = 0,
		kHardStand     = 1,
		kSoftStand     = 2,
		kTrot          = 3,
		kPace          = 4,
		kJumpTrot      = 5,
		kBound         = 6,
		kPronk        = 7,
		kHeadUp        = 8,
		kHipUp         = 9,
		kLoseControl   = 10,
	};
}

#endif /* PERFORMANCE_STATES_H_ */
