/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  static const int shooterMotorLeftID = 6;
  static const int shooterMotorRightID = 7;
  static const int feederMotorID = 3;
  ctre::phoenix::motorcontrol::can::TalonFX shooterMotorLeft{shooterMotorLeftID};
  ctre::phoenix::motorcontrol::can::TalonFX shooterMotorRight{shooterMotorRightID};
  ctre::phoenix::motorcontrol::can::TalonSRX m_feederMotor{feederMotorID};

  frc::XboxController m_stick{0};

  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;
  const double MaxRPM = 6380;
  double SetPoint = 0;
  double sensorResolution = 4096;
  double gearRatio = 1/2;

  int ConvertRPMToTicksPer100Ms(double rpm);
  double ConvertTicksPer100MsToRPM(int ticksPer100ms);
};
