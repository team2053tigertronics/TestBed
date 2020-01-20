/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <rev/CANSparkMax.h>
#include <frc/XboxController.h>

void Robot::RobotInit() {
    shooterMotorLeft.ConfigFactoryDefault();
    shooterMotorRight.ConfigFactoryDefault();

    shooterMotorRight.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
    shooterMotorRight.Config_kF(0, kFF);
    shooterMotorRight.Config_kP(0, kP);
    shooterMotorRight.Config_kI(0, kP);
    shooterMotorRight.Config_kD(0, kP);
    shooterMotorRight.Config_IntegralZone(0, kIz);
    shooterMotorRight.ConfigPeakOutputForward(kMaxOutput);
    shooterMotorRight.ConfigPeakOutputReverse(kMinOutput);

    shooterMotorLeft.Follow(shooterMotorRight);
    shooterMotorLeft.SetInverted(true);

    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

    SetPoint = 0.0;
}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
    
}

void Robot::TeleopPeriodic() {
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);

    if((p != kP)) { shooterMotorRight.Config_kP(0, p); kP = p; }
    if((i != kI)) { shooterMotorRight.Config_kI(0, i); kI = i; }
    if((d != kD)) { shooterMotorRight.Config_kD(0, d); kD = d; }
    if((iz != kIz)) { shooterMotorRight.Config_IntegralZone(0, iz); kIz = iz; }
    if((ff != kFF)) { shooterMotorRight.Config_kF(0, ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      shooterMotorRight.ConfigPeakOutputForward(max);
      shooterMotorRight.ConfigPeakOutputReverse(min);
      kMinOutput = min; kMaxOutput = max; 
    }

    if(m_stick.GetYButtonPressed()) {
        SetPoint += 500;
    }
    if(m_stick.GetAButtonPressed()) {
        SetPoint -= 500;
    }
    if(m_stick.GetBButtonPressed()) {
        SetPoint += 200;
    }
    if(m_stick.GetXButtonPressed()) {
        SetPoint -= 200;
    }
    if(m_stick.GetBackButtonPressed()) {
        SetPoint = 0;
    }

    bool leftPressed = m_stick.GetBumper(frc::XboxController::JoystickHand::kLeftHand);
    bool rightPressed = m_stick.GetBumper(frc::XboxController::JoystickHand::kRightHand);
    if(leftPressed) 
    {
      m_feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
    }
    if(rightPressed)
    {
      m_feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
    }
    if(!leftPressed && !rightPressed) {
      m_feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }

    shooterMotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, ConvertRPMToTicksPer100Ms(SetPoint));

    frc::SmartDashboard::PutNumber("Commanded Shooter Velocity", SetPoint);
    frc::SmartDashboard::PutNumber("Left Motor Velocity", ConvertTicksPer100MsToRPM(shooterMotorLeft.GetSelectedSensorVelocity()));
    frc::SmartDashboard::PutNumber("Right Motor Velocity", ConvertTicksPer100MsToRPM(shooterMotorRight.GetSelectedSensorVelocity()));
}

void Robot::TestPeriodic() {

}

int Robot::ConvertRPMToTicksPer100Ms(double rpm) {
    return (rpm  / 600) * (sensorResolution / gearRatio);
}

double Robot::ConvertTicksPer100MsToRPM(int ticksPer100ms) {
    return (ticksPer100ms * 600) / (sensorResolution / gearRatio);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
