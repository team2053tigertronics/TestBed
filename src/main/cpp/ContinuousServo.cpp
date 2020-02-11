/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ContinuousServo.h"
#include "frc/smartdashboard/SendableRegistry.h"

ContinuousServo::ContinuousServo(int channel) : frc::PWMSpeedController(channel) {
    SetBounds(1.9, 1.902, 1.55, 1.102, 1.1);
    SetPeriodMultiplier(frc::PWMSpeedController::kPeriodMultiplier_1X);
    SetSpeed(0.0);
    SetZeroLatch();
}
