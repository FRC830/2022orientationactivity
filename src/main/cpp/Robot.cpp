// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Teleop.h"
#include <iostream>
#include <math.h>

using namespace std;

void Robot::RobotInit() {
  motorGroupLeft.SetInverted(true);

  frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic() {}


void Robot::AutonomousInit() {

  motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  firstCallToAuton = true;
  
  autonMovingMotor = false;
  autonStep = 1;
  newAutonCall = true;


  motorFLEncoder.SetPosition(0.0);
  motorFREncoder.SetPosition(0.0);
  motorBLEncoder.SetPosition(0.0);
  motorBREncoder.SetPosition(0.0);

  if (motorBREncoder.SetPosition(0.0) == rev::REVLibError::kOk)
  {
    std::cout << std::endl << "BR Encoder successfully set to: " << motorBREncoder.GetPosition() << std::endl;
  }
  std::cout << "FL Encoder " << motorFLEncoder.GetPosition() << std::endl;
  std::cout << "BL Encoder " << motorBLEncoder.GetPosition() << std::endl;
  std::cout << "FR Encoder " << motorFREncoder.GetPosition() << std::endl;

  // look at suffleboard...

  leftFlywheelTalon.Set(TalonFXControlMode::Velocity, 0);
  rightFlywheelTalon.Set(TalonFXControlMode::Follower, leftFlywheelTalon.GetDeviceID());
  backSpinTalon.Set(TalonFXControlMode::Velocity, 0);
  rightFlywheelTalon.SetInverted(true);
  backSpinTalon.SetInverted(true);

  
  leftVictor.Set(VictorSPXControlMode::PercentOutput, 0);
  middleVictor.Set(VictorSPXControlMode::PercentOutput, -0);
  rightVictor.SetInverted(true);
  rightVictor.Set(VictorSPXControlMode::Follower, leftVictor.GetDeviceID());

  climber.Set(0);
  
}

void Robot::AutonomousPeriodic() {

}


void Robot::TeleopInit() {
  pilot.setDeadzone();
  copilot.setDeadzone();
  pilot.setDeadzoneLimit(0.1);
  copilot.setDeadzoneLimit(0.1);
  pilot.setSensitivity();
  pilot.setSensitivityLevel(defaultInputSensitivity);

  if (ebrake)
  {
  motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  }

  climber.Set(0);
}

void Robot::TeleopPeriodic() {
  drivetrain.ArcadeDrive(pilot.GetRightY(), pilot.GetLeftX(), true);
}

void Robot::DisabledInit() {

  motorFR.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  motorBR.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  motorBL.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  motorFL.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif