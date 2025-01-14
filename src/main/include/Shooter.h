#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/drive/DifferentialDrive.h"
#include <frc/XboxController.h> 
#include "ctre/Phoenix.h"
#include "Take.h"

class Shooter {
 public:
  Shooter(frc::DifferentialDrive* d, frc::XboxController* s, Take* t);
  
  void Fire();
  void Reset();
  void InitializePIDControllers();
  void InitializeDashboard();
  void ReadDashboard();
  void ManualShoot(); 
  void Dump();

 private:

  double CalculateRPM(double d);
  bool LimelightTracking();

  frc::DifferentialDrive* m_drive;
  frc::XboxController*    m_stick;
  Take*                   m_take;

  double m_overrideRPM;
  double kHeightOfTarget   = 103.0; // TODO: Measure
  double kHeightLimelight  = 28.0;  // TODO: Measure
  double kLimelightAngle   = 13.861;  // TODO: Measure
  double kRadiusOfTarget   = 26.7;


  WPI_TalonFX m_shootingMotorAlpha {21};
  WPI_TalonFX m_shootingMotorBeta {20};

  TalonFXSensorCollection m_shootingMotorAlphaEncoder = m_shootingMotorAlpha.GetSensorCollection();
  TalonFXSensorCollection m_shootingMotorBetaEncoder = m_shootingMotorBeta.GetSensorCollection();

  struct pidCoeff {
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  };

  pidCoeff m_shooterCoeff{0.35, 0.000002, 0.0, 0.0, 0.06, 1.0, -1.0};

  std::shared_ptr<nt::NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-brute");


};
