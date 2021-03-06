/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/smartdashboard/SendableRegistry.h"
#include "ContinuousServo.h"

ContinuousServo::ContinuousServo(int channel) : frc::PWMSpeedController(channel) {
    SetBounds(1.9, 1.89, 1.550, 1.11, 1.1);
    SetPeriodMultiplier(frc::PWMSpeedController::kPeriodMultiplier_4X);
    SetSpeed(0.0);
    derp = 0;
    //SetZeroLatch();
}

