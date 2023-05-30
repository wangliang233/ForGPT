/*
*  Copyright (c) 2018-- Deep Robotics. All rights reserved.
*  License(GPL)
*  Developer:
*    DeepRobotics
*/

#ifndef INTERFACE_VERSION_RAS_H_
#define INTERFACE_VERSION_RAS_H_

#define  DEEPRAS_VERSION_INTERFACE 2
#define  DEEPRAS_VERSION_MINOR 1
#define  DEEPRAS_VERSION_PATCH 5
#define  DEEPRAS_VERSION_BUILD 6
#define  DEEPRAS_VERSION_NAME "DeepRAS Version 2.1.5(6)"
#define  DEEPRAS_VERSION ((DEEPRAS_VERSION_INTERFACE<<24) | \
                         (DEEPRAS_VERSION_MINOR<<16) | \
                         (DEEPRAS_VERSION_PATCH<<8 ) | \
                         DEEPRAS_VERSION_BUILD)

//const char* const kDEEPRASVersionQualifier;

/// This file indicate the version of deepras.
/// Recently will not used in API build.
namespace deepras {

	const char* const kTargetRobot = "JY02";
	const char* const kTargetSystem = "0001";

	const char* const   kDeeprasVersionString = DEEPRAS_VERSION_NAME;
	const unsigned char kDeeprasVersionMajor = DEEPRAS_VERSION_INTERFACE;
	const unsigned char kDeeprasVersionMinor = DEEPRAS_VERSION_MINOR;
	const unsigned char kDeeprasVersionPatch = DEEPRAS_VERSION_PATCH;
	const unsigned char kDeeprasVersionBuild = DEEPRAS_VERSION_BUILD;
	//const char* const kDeeprasVersionQualifier;
	const unsigned int kDeeprasVersionHex = DEEPRAS_VERSION;

}
#endif // INTERFACE_VERSION_RAS_H_
