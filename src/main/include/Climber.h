#pragma once

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/controller/PIDController.h>


class Climber {
public:
  //ClimberSubsystem();
  void ClimberPIDInit();
  void ClimberDashRead();
  void ClimberDashInit();
  void ExtendLeft(double step);
  void ExtendRight(double step);
  void RotateLeft(double step);
  void RotateRight(double step);

private:
  frc::SendableChooser<std::string> m_chooser;


  // Stolen from Robot.h
  static const int rightClimberRotationNeoDeviceID = 10;
  static const int leftClimberRotationNeoDeviceID = 11;

  rev::CANSparkMax m_rightClimberRotationNeo{rightClimberRotationNeoDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftClimberRotationNeo{leftClimberRotationNeoDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  // Climber falcons
  WPI_TalonFX m_leftClimberExtender = {12};
  WPI_TalonFX m_rightClimberExtender = {13};
  TalonFXSensorCollection m_leftClimberExtenderEncoder = m_leftClimberExtender.GetSensorCollection();
  TalonFXSensorCollection m_rightClimberExtenderEncoder = m_rightClimberExtender.GetSensorCollection();

  // Ding dong, you are wrong
  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };
  
  rev::SparkMaxPIDController m_leftClimberRotatePIDController = m_leftClimberRotationNeo.GetPIDController();
  rev::SparkMaxPIDController m_rightClimberRotatePIDController = m_rightClimberRotationNeo.GetPIDController();

  //thing
  
  rev::SparkMaxRelativeEncoder m_rightClimberEncoder = m_rightClimberRotationNeo.GetEncoder(); 
  rev::SparkMaxRelativeEncoder m_leftClimberEncoder = m_leftClimberRotationNeo.GetEncoder(); 

  //Neos
  pidCoeff m_leftClimberRotateCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pidCoeff m_rightClimberRotateCoeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


  //splish splash, your opinion is trash

  //Falcons 
  pidCoeff m_leftClimberExtendCoeff{0.1, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0}; 
  pidCoeff m_rightClimberExtendCoeff{0.1, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0};

  //vars

  double m_rotatePointR = 0.0;
  double m_rotatePointL = 0.0;

  double m_climbExtendPointR = 0.0;
  double m_climbExtendPointL = 0.0;
};


