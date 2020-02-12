/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <algorithm>

int ENCODER_MAX_VAL = 1000; //CHANGE ME

double map(double input_start, double input_end, double output_start, double output_end, double input) {
    double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
    double output = output_start + slope * (input - input_start);
    return output;
}

double GetHoodAngle(double encoderVal) {
    return map(0, ENCODER_MAX_VAL, 0, 90, encoderVal);
}

void Robot::RobotInit() {
    shooterMotorLeft.ConfigFactoryDefault();
    shooterMotorRight.ConfigFactoryDefault();
    intakeMotor.ConfigFactoryDefault();
    feederMotor.ConfigFactoryDefault();
    conveyorMotor.ConfigFactoryDefault();

    shooterMotorRight.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
    shooterMotorRight.Config_kF(0, kFF);
    shooterMotorRight.Config_kP(0, kP);
    shooterMotorRight.Config_kI(0, kP);
    shooterMotorRight.Config_kD(0, kP);
    shooterMotorRight.Config_IntegralZone(0, kIz);
    shooterMotorRight.ConfigPeakOutputForward(kMaxOutput);
    shooterMotorRight.ConfigPeakOutputReverse(0);

    shooterMotorLeft.Follow(shooterMotorRight);
    shooterMotorLeft.SetInverted(true);

    encoderCAN.ConfigFactoryDefault();
    encoderCAN.SetQuadraturePosition(0);

    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("Servo Value", kServoValue);
    frc::SmartDashboard::PutNumber("Hood P", hoodP);
    frc::SmartDashboard::PutNumber("Hood I", hoodI);
    frc::SmartDashboard::PutNumber("Hood D", hoodD);

    SetPoint = 0.0;
    otherSet = 0.0;
    conveyorSet = 0.0;
    feederSet = 0.0;
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
    double hoodp = frc::SmartDashboard::GetNumber("Hood P", 0);
    double hoodi = frc::SmartDashboard::GetNumber("Hood I", 0);
    double hoodd = frc::SmartDashboard::GetNumber("Hood D", 0);

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
    if((hoodp != hoodP)) { hoodController.SetP(hoodp); hoodP = hoodp; }
    if((hoodi != hoodI)) { hoodController.SetI(hoodi); hoodI = hoodi; }
    if((hoodd != hoodD)) { hoodController.SetD(hoodd); hoodD = hoodd; }

    int currentEncoderVal = encoderCAN.GetQuadraturePosition();

    if(m_stick.GetYButtonPressed()) {
        SetPoint = SetPoint + 500;
    }
    if(m_stick.GetAButtonPressed()) {
        kServoValue = kServoValue + .1;
    }
    if(m_stick.GetBButtonPressed()) {
        kServoValue = kServoValue - .1;
    }
    /*if(m_stick.GetAButtonPressed()) {
        otherSet = 0;
        conveyorSet = 0;
    }*/
    /*if(m_stick.GetBButtonPressed()) {
        otherSet = .5;
        conveyorSet = .5;
    }*/
    if(m_stick.GetXButtonPressed()) {
        SetPoint = 0;
    }
    if(m_stick.GetStartButtonPressed()) {
        feederSet = .5;
    }
    if(m_stick.GetStartButtonReleased()) {
        feederSet = 0;
    }
    if(m_stick.GetBackButtonPressed()) {
        conveyorSet = 0;
        feederSet = 0;
        otherSet = 0;
        SetPoint = 0;
        kServoValue = 0;
    }

/*
    //bool leftPressed = m_stick.GetBumper(frc::XboxController::JoystickHand::kLeftHand);
    //bool rightPressed = m_stick.GetBumper(frc::XboxController::JoystickHand::kRightHand);
    if(m_stick.GetAButtonPressed()) 
    {
      //m_feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
      kServoValue = .9;
      SetServoSpeed(kServoValue);

    }
    else if(m_stick.GetBButtonPressed())
    {
      //m_feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
      kServoValue = -.9;
      SetServoSpeed(kServoValue);
    }
    else {
      //m_feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      kServoValue = .1;
      SetServoSpeed(kServoValue);
    }
    */
    //SetServoSpeed(kServoValue);

    //THIS HAS TO BE CALLED EVERY LOOP YOU DINGUS
    //MEANING DONT PUT THIS IS AN IF STATEMENT
    double pidoutput = hoodController.Calculate(GetHoodAngle(currentEncoderVal));
    if(m_stick.GetBumper(frc::XboxController::kLeftHand)) {
        hoodServo.Set(std::clamp(pidoutput, -1.0, 1.0));
    }
    else {
        hoodServo.Set(kServoValue);
    }
    //hoodServo.Set(kServoValue);
    shooterMotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, (int)ConvertRPMToTicksPer100Ms(SetPoint));
    //intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -otherSet);
    feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, feederSet);
    //conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, conveyorSet);

    frc::SmartDashboard::PutNumber("CAN ENCODER", currentEncoderVal);
    frc::SmartDashboard::PutNumber("Commanded Shooter Velocity", SetPoint);
    frc::SmartDashboard::PutNumber("Left Motor Velocity", ConvertTicksPer100MsToRPM(shooterMotorLeft.GetSelectedSensorVelocity()));
    frc::SmartDashboard::PutNumber("Right Motor Velocity", ConvertTicksPer100MsToRPM(shooterMotorRight.GetSelectedSensorVelocity()));
    frc::SmartDashboard::PutNumber("Hood Angle", GetHoodAngle(currentEncoderVal));
}

void Robot::TestPeriodic() {

}

int Robot::ConvertRPMToTicksPer100Ms(double rpm) {
    return (rpm  / 600.0) * sensorResolution;
}

double Robot::ConvertTicksPer100MsToRPM(int ticksPer100ms) {
    return (ticksPer100ms * 600) / sensorResolution;
}

void Robot::SetServoSpeed(double percent) {
    frc::SmartDashboard::PutNumber("INSIDE LOOP: PERCENT", percent);
    //frc::SmartDashboard::PutNumber("INSIDE LOOP: SERVO COMMAND", )
    hoodServo.Set(percent);
    //hoodServo.derp++;
    //frc::SmartDashboard::PutNumber("DERP: ", hoodServo.derp);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
