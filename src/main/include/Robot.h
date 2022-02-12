// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "Climber.h"
#include "Shooter.h"
#include "Take.h"

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>


#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/GenericHID.h>
#include <frc/drive/RobotDriveBase.h>
#include <frc/RobotController.h>
#include <frc/motorcontrol/MotorController.h>
#include <frc/DoubleSolenoid.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include "frc/drive/RobotDriveBase.h"
#include "frc/drive/DifferentialDrive.h"
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/controller/PIDController.h>

#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/controller/SimpleMotorFeedforward.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void LimelightTracking();
  double CalculateRPM(double d);
  //void ShooterAim();

  void ShooterArm();
  void ShooterFire();
  void IntakeDeploy();
  void IntakeReturn();

  void InitializePIDControllers();
  void InitializeDashboard();
  void ReadDashboard();


 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  const std::string kThreeBallBlue = "ThreeBallBlue";
  const std::string kTwoBallBlue = "TwoBallBlue";
  const std::string kThreeBallRed = "ThreeBallRed";
  const std::string kTwoBallRed = "TwoBallRed";
  std::string m_autoSelected;

  frc::RamseteController controller1;

  // void autoFollowPath();
  // void autoDrive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot);
  // void setSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);

  // units::meter_t kTrackWidth = 0.478028_m;  

  // static constexpr auto   kS = 0.27_V;                         
  // static constexpr auto   kV = 1.53 * 1_V * 1_s / 1_m;         
  // static constexpr auto   kA = 0.254 * 1_V * 1_s * 1_s / 1_m; 

  // frc::DifferentialDriveOdometry m_odometry;
  // frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  // frc::SimpleMotorFeedforward<units::meters> m_feedforward{kS, kV, kA};

  double m_driveExponent = 1.2;
  double m_turnFactor = 0.6;
  bool shootMan;
  bool limelightTrackingBool = false;
  bool wrongBall;
  bool uptakeBool;
  fs::path deployDirectory;

  double taLowBound, taHighBound;
  double txLowBound, txHighBound;
  double tyLowBound, tyHighBound;
  double heightOfTarget;
  double heightLimelight;
  double constantLimelightAngle;

  Climber m_climber;
  Take m_take;

//So long, Joystick.h!
  frc::XboxController m_stick{0};
// A second controler
  frc::XboxController m_stick_climb{1};

  WPI_TalonFX m_frontRightMotor = {1};
  WPI_TalonFX m_midRightMotor = {15};
  WPI_TalonFX m_backRightMotor = {2};
  WPI_TalonFX m_frontLeftMotor = {3};
  WPI_TalonFX m_midLeftMotor = {16};
  WPI_TalonFX m_backLeftMotor = {4};

  // Left side of the robot is inverted
  // Tonk drive
  frc::MotorControllerGroup m_leftDrive{m_frontLeftMotor, m_midLeftMotor, m_backLeftMotor};
  frc::MotorControllerGroup m_rightDrive{m_frontRightMotor, m_midRightMotor, m_backRightMotor};

  frc::DifferentialDrive m_drive{m_leftDrive, m_rightDrive};

  
// I don't know what either of theese do
  static const int uptakeMotorDeviceID = 9;
  static const int uptakeIdleMotorDeviceID = 14;
  

//auto timer
frc::Timer autoTimer;

frc::Trajectory m_trajectory;

Shooter m_shooter{&m_drive, &m_stick, &m_take};

};
