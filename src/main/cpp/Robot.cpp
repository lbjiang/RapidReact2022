// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


// Standard C++ Libraries
#include <iostream>
#include <math.h>

// FIRST Specific Libraries
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * Initialization method for Robot. Call Subsystem initialization methods here
 * in addition to setting up dashboard
 */
void Robot::RobotInit() {
  
  // Setup initiate Diff Drive object with an initial heading of zero
  m_odometry = new frc::DifferentialDriveOdometry(frc::Rotation2d(0_deg));
  

  InitializePIDControllers(); 
  InitializeDashboard();

  // Setup Autonomous options
  m_chooser.SetDefaultOption(kAutoDefault, kAutoDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_chooser.AddOption(kThreeBallPath1, kThreeBallPath1);
  m_chooser.AddOption(kThreeBallPath2, kTwoBallPath2);
  m_chooser.AddOption(kThreeBallPath3, kThreeBallPath3);
  m_chooser.AddOption(kTwoBallPath1, kTwoBallPath1);
  m_chooser.AddOption(kTwoBallPath2, kTwoBallPath2);
  
  // Add Autonomous options to dashboard
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  
  // Invert left side of drive train (if needed due to construction)
  m_leftDrive.SetInverted(true);
  
  // TODO: Add break ball functionality once available 

  manualShootingEnabled = true;
  wrongBallInSystem = false;

  // Setup mid motor pair
  m_midRightMotor.Follow(m_frontRightMotor);
  m_midLeftMotor.Follow(m_frontLeftMotor);

  //Setup back motor pair
  m_backRightMotor.Follow(m_frontRightMotor);
  m_backLeftMotor.Follow(m_frontLeftMotor);

  m_drive.SetSafetyEnabled(false);

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {

  // Get choosen autonomous mode
  m_autoSelected = m_chooser.GetSelected();

  // Print out the selected autonomous mode
  fmt::print("Auto selected: {}\n", m_autoSelected);

  // TODO: Make the following a switch statement
  if (m_autoSelected == kThreeBallPath1) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "Paths" / "Patheaver/Paths/ThreeBallFirst.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kThreeBallPath2) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/ThreeBallSecond.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kThreeBallPath3) { 
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/ThreeBallThird.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kTwoBallPath1) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/TwoBallFirst.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

  if (m_autoSelected == kTwoBallPath2) {
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "autos" / "Patheaver/autos/TwoBallSecond.wpilib.json";
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  }

}

/**
 * This is the method called every 20ms (by default, can be changed)
 * during the autonomous period
 */
void Robot::AutonomousPeriodic() {
/*
  // Check if current auto mode is the custom auto mode
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  // Iteration one
  /*
   autoTimer.Start();
   if (autoTimer.Get() <= units::time::second_t(4)) {
     m_shooter.Fire();
   }
   if (autoTimer.Get() > units::time::second_t(4) && autoTimer.Get() <= units::time::second_t(5)) {
     m_drive.ArcadeDrive(0.5,0);
   }
  */

  // Iteration two
 
  autoTimer.Start();
  if (autoTimer.Get() <= units::time::second_t(1)) {
    m_take.DeployIntake();
  }
  if (autoTimer.Get() > units::time::second_t(1) && autoTimer.Get() <= units::time::second_t(5)) {
    m_take.AutoRunIntake(1);
    m_drive.ArcadeDrive(0.5, 0);
  }
  if  (autoTimer.Get() > units::time::second_t(5) && autoTimer.Get() <= units::time::second_t(9)) {
    m_shooter.Fire();
  }
  if  (autoTimer.Get() > units::time::second_t(9) && autoTimer.Get() <= units::time::second_t(12)) {
    m_shooter.Fire();
  }
  
}

/**
 * This is the method called at the beginning of teleoperated mode
 */
void Robot::TeleopInit() {

  m_alliance = frc::DriverStation::GetAlliance();
   InitializePIDControllers();
  ReadDashboard();
}

void Robot::TeleopPeriodic() {
  // Intake
  m_take.Run(m_stick.GetLeftBumperReleased(), m_stick.GetRightBumper(), m_alliance);
  
  double a = .375/.4495;
  double b = .0745/.4495;
  //Read controller input
  double throttle = -m_stick.GetLeftTriggerAxis() + 0.9 * m_stick.GetRightTriggerAxis();
 
  double throttleExp = a * pow(m_stick.GetLeftTriggerAxis(), 4) + b * pow(m_stick.GetLeftTriggerAxis(), 1.48)-a * pow(m_stick.GetRightTriggerAxis(), 4) + b * pow(m_stick.GetRightTriggerAxis(), 1.48);
  // double turnInput = pow(m_stick.GetLeftX()*m_turnFactor,1.72) - pow(m_stick.GetLeftY()*m_turnFactor,1.72);
  double turnInput = m_stick.GetLeftX() - m_stick.GetLeftY();
  // Shooter
  if (m_stick.GetRightBumper()) {
    m_shooter.Fire();
  } else {
    m_drive.ArcadeDrive(throttle, turnInput);
  }
  if (m_stick.GetRightBumperReleased()) {
    m_shooter.Reset();
  }

}

// This method is called at the beginning of the disabled state
void Robot::DisabledInit() {}

// This method is called every 20ms (by default) during disabled
void Robot::DisabledPeriodic() {}


// This method is called at the beginning of the testing state
void Robot::TestInit() {}

// This method is called every 20ms (by default) during testing
void Robot::TestPeriodic() {}


// Method for initializing PID Controller
void Robot::InitializePIDControllers() {
  //  m_climber.ClimberPIDInit();
  //  m_take.TakePIDInit();
  //  m_shooter.InitializePIDControllers();

}

// Method for initializing the Dashboard
void Robot::InitializeDashboard() {
  m_climber.ClimberDashInit();
  m_take.TakeDashInit();
  m_shooter.InitializeDashboard();
}

// Method for reading the Dashboard
void Robot::ReadDashboard() {
  m_climber.ClimberDashRead();
  m_take.TakeDashRead();
  m_shooter.ReadDashboard();
}

// Method for determining speeds during autonomous
void Robot::setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {

  // QUESTION: Why are these consts? Const in this context means you can't change
  // it after it has been set, but since these variables are local in scope and
  // not changed anywheres after that doesn't make much sense
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
 
  // QUESTION: Same as above
  const double leftOutput = m_frontRightMotorPIDController.Calculate(m_frontRightMotor.GetActiveTrajectoryVelocity(), speeds.left.to<double>());
  const double rightOutput = m_frontLeftMotorPIDController.Calculate(m_frontLeftMotor.GetActiveTrajectoryVelocity(), speeds.right.to<double>());
 
  // Set the voltages for the left and right drive motor groups
  m_leftGroup->SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup->SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
 
}

// QUESTION: Why is this a seperate method? Since you are just passing input
// parameters to a method and nothing else, this is just not needed
// Unless there is a plan to add more complexity/logic to THIS specfic function
void Robot::autoDrive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot){
  setSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

// This method gets called every teleop period and follows the predetermined
// autonomous paths based on auto state
void Robot::autoFollowPath(){

  // QUESTION: Why all the auto types?

  // If the robot is still within the trajectory time frame
  if (autoTimer.Get() < m_trajectory.TotalTime()) {
    // Get desired pose
    auto desiredPose = m_trajectory.Sample(autoTimer.Get());
    // Get desired speeds from current pose vs desired pose
    auto refChassisSpeeds = controller1.Calculate(m_odometry->GetPose(), desiredPose);
    
    // Drive based on desired speeds
    autoDrive(refChassisSpeeds.vx, refChassisSpeeds.omega);
  }
  else {
    // Stop the robot
    autoDrive(0_mps, 0_rad_per_s);
  }
}

// If we are not running in test mode
#ifndef RUNNING_FRC_TESTS
int main() {
  // Start the robot
  return frc::StartRobot<Robot>();
}
#endif
