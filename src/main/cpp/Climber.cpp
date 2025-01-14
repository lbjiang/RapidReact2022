#include "Climber.h"

// Standard C++ Libraries
#include <iostream>
#include <math.h>

// FIRST Specific Libraries
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

// TODO: Add a kill button

// Initialization
Climber::Climber() {
  // ClimberDashInit();
  ClimberPIDInit();
}

// NOTE: This class is missing a constructor method

/**
 * This method Extends/Lowers the left climber to the set point
 * 
 * parameters:
 *   setPointL - double representing the point to move towards
 */
void Climber::ExtendALowerL(double setPointL) {
  
  m_climbExtendPointL = setPointL;
  m_leftClimberExtender.Set(ControlMode::MotionMagic, m_climbExtendPointL*2048.0);
}

/**
 * This method Extends/Lowers the right climber to the set point
 * 
 * parameters:
 *   setPointR - double representing the point to move towards
 */
void Climber::ExtendALowerR(double setPointR) {
  m_climbExtendPointR = setPointR;
  m_rightClimberExtender.Set(ControlMode::MotionMagic, m_climbExtendPointR*2048.0);
}

/**
 * This method rotates the left climber to the set point
 * 
 * parameters:
 *   rotatePointL - double representing the angle to rotate to
 */
void Climber::RotateLeft(double rotatePointL){
  m_rotateSetpointL = rotatePointL;
 m_leftClimberRotatePIDController.SetReference(m_rotateSetpointL, rev::CANSparkMax::ControlType::kSmartMotion);
}

/**
 * This method rotates the right climber to the set point
 * 
 * parameters:
 *   rotatePointR - double representing the angle to rotate to
 */
void Climber::RotateRight(double rotatePointR){
  m_rotateSetpointR = rotatePointR;
  m_rightClimberRotatePIDController.SetReference(m_rotateSetpointR, rev::CANSparkMax::ControlType::kSmartMotion);
}

void Climber::EngageLeft(double throttle) {
  //could set overall (max) soft limits here? Also, should do throttle?? Or do velocity PIDs??
  m_leftClimberExtender.Set(throttle);
  //std::cout << "Left Encoder Position: " << m_leftClimberExtender.GetSelectedSensorPosition() << "\n";
}

void Climber::EngageRight(double throttle) {
  m_rightClimberExtender.Set(throttle);
  //std::cout << "Right Encoder Position: " << m_rightClimberExtender.GetSelectedSensorPosition() << "\n";
}

/**
 * Method to initialize the climber dashboard
 */
void Climber::ClimberDashInit(){

  // PID Constants for Left Climber Extension
  frc::SmartDashboard::PutNumber("Left Climber Extend P Gain", m_leftClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Left Climber Extend I Gain", m_leftClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Left Climber Extend D Gain", m_leftClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Left Climber Extend FF Gain", m_leftClimberExtendCoeff.kFF);
  frc::SmartDashboard::PutNumber("Left Climber Extend Min Output", m_leftClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Left Climber Extend Max Output", m_leftClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Left Climber Extend Point", m_climbExtendPointL);

  // PID Constants for Right Climber Extension
  frc::SmartDashboard::PutNumber("Right Climber Extend P Gain", m_rightClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Right Climber Extend I Gain", m_rightClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Right Climber Extend D Gain", m_rightClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Right Climber Extend FF Gain", m_rightClimberExtendCoeff.kFF);
  frc::SmartDashboard::PutNumber("Right Climber Extend Min Output", m_rightClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Right Climber Extend Max Output", m_rightClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Right Climber Extend Point", m_climbExtendPointR);

  // PID Constants for Left Climber Rotation
  frc::SmartDashboard::PutNumber("Left Climber Rotate P Gain", m_leftClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Left Climber Rotate I Gain", m_leftClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Left Climber Rotate D Gain", m_leftClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Max Output", m_leftClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Min Output", m_leftClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Point", m_rotateSetpointL);

  // PID Constants for Right Climber Rotation
  frc::SmartDashboard::PutNumber("Right Climber Rotate P Gain", m_rightClimberExtendCoeff.kP);
  frc::SmartDashboard::PutNumber("Right Climber Rotate I Gain", m_rightClimberExtendCoeff.kI);
  frc::SmartDashboard::PutNumber("Right Climber Rotate D Gain", m_rightClimberExtendCoeff.kD);
  frc::SmartDashboard::PutNumber("Right Climber Rotate Max Output", m_rightClimberExtendCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Right Climber Rotate Min Output", m_rightClimberExtendCoeff.kMinOutput);
  frc::SmartDashboard::PutNumber("Right Climber Rotate Point", m_rotateSetpointR);
}

/**
 * Method to read the climber dashboard values
 */
void Climber::ClimberDashRead(){

  // Create variables to hold PID values (p, i, d, min, and max)
  double p, i, d, min, max;

  // Read right climber rotation PID constants
  p   = frc::SmartDashboard::GetNumber("Right Climber Rotate P Gain", 0.0);
  i   = frc::SmartDashboard::GetNumber("Right Climber Rotate I Gain", 0.0);
  d   = frc::SmartDashboard::GetNumber("Right Climber Rotate D Gain", 0.0);
  min = frc::SmartDashboard::GetNumber("Right Climber Rotate Min Output", 0.0);
  max = frc::SmartDashboard::GetNumber("Right Climber Rotate Max Output", 0.0);
  m_rotateSetpointL = frc::SmartDashboard::GetNumber("Left Climber Rotate Point", 0.0);
  m_rotateSetpointR = frc::SmartDashboard::GetNumber("Right Climber Rotate Point", 0.0);

  // Print PID to console
  std::cout << "Read Dashboard Right Rotate Climber p gain: " << p << "\n";
  std::cout << "Read Dashboard Right Climber Rotate i gain: " << i << "\n";
  std::cout << "Read Dashboard Right Climber Rotate d gain: " << d << "\n";

  // If any PID value is different from currently stored, set that value
  if ((p != m_rightClimberRotateCoeff.kP)) { m_rightClimberRotatePIDController.SetP(p);m_rightClimberRotateCoeff.kP = p; }
  if ((i != m_rightClimberRotateCoeff.kI)) { m_rightClimberRotatePIDController.SetI(i); m_rightClimberRotateCoeff.kI = i; }
  if ((d != m_rightClimberRotateCoeff.kD)) { m_rightClimberRotatePIDController.SetD(d); m_rightClimberRotateCoeff.kD = d; }
  if ((max != m_rightClimberRotateCoeff.kMaxOutput) || (min != m_rightClimberRotateCoeff.kMinOutput)) { 
    m_rightClimberRotatePIDController.SetOutputRange(min, max); 
    m_rightClimberRotateCoeff.kMinOutput = min; m_rightClimberRotateCoeff.kMaxOutput = max; 
  }

  // Read PID constants for left climber rotate
  p   = frc::SmartDashboard::GetNumber("Left Climber Rotate P Gain", 0.0);
  i   = frc::SmartDashboard::GetNumber("Left Climber Rotate I Gain", 0.0);
  d   = frc::SmartDashboard::GetNumber("Left Climber Rotate D Gain", 0.0);
  min = frc::SmartDashboard::GetNumber("Left Climber Rotate Min Output", 0.0);
  max = frc::SmartDashboard::GetNumber("Left Climber Rotate Max Output", 0.0);

  // Print PID constants for left climber rotate
  std::cout << "Read Dashboard left Climber Rotate p gain: " << p << "\n";
  std::cout << "Read Dashboard left Climber Rotate i gain: " << i << "\n";
  std::cout << "Read Dashboard left Climber Rotate d gain: " << d << "\n";

  // If any PID constants for left climber rotate differ from stored values, set the new value
  if ((p != m_leftClimberRotateCoeff.kP)) { m_leftClimberRotatePIDController.SetP(p);m_leftClimberRotateCoeff.kP = p; }
  if ((i != m_leftClimberRotateCoeff.kI)) { m_leftClimberRotatePIDController.SetI(i); m_leftClimberRotateCoeff.kI = i; }
  if ((d != m_leftClimberRotateCoeff.kD)) { m_leftClimberRotatePIDController.SetD(d); m_leftClimberRotateCoeff.kD = d; }
  if ((max != m_leftClimberRotateCoeff.kMaxOutput) || (min != m_leftClimberRotateCoeff.kMinOutput)) { 
    m_leftClimberRotatePIDController.SetOutputRange(min, max); 
    m_leftClimberRotateCoeff.kMinOutput = min; m_leftClimberRotateCoeff.kMaxOutput = max; 
  }

  // Read left climber extend/retract PID values
  m_leftClimberExtendCoeff.kP   = frc::SmartDashboard::GetNumber("Left Climber Extend P Gain", 0.0);
  m_leftClimberExtendCoeff.kI   = frc::SmartDashboard::GetNumber("Left Climber Extend I Gain", 0.0);
  m_leftClimberExtendCoeff.kD   = frc::SmartDashboard::GetNumber("Left Climber Extend D Gain", 0.0);
  m_leftClimberExtendCoeff.kFF  = frc::SmartDashboard::GetNumber("Left Climber Extend FF Gain", 0.0);
  m_leftClimberExtendCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Left Climber Extend Min Output", 0.0);
  m_leftClimberExtendCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Left Climber Extend Max Output", 0.0);
  m_climbExtendPointL = frc::SmartDashboard::GetNumber("Left Climber Extend Point", 0.0);

  // Read right climber extend/retract PID values
  m_rightClimberExtendCoeff.kP   = frc::SmartDashboard::GetNumber("Right Climber Extend P Gain", 0.0);
  m_rightClimberExtendCoeff.kI   = frc::SmartDashboard::GetNumber("Right Climber Extend I Gain", 0.0);
  m_rightClimberExtendCoeff.kD   = frc::SmartDashboard::GetNumber("Right Climber Extend D Gain", 0.0);
  m_rightClimberExtendCoeff.kFF  = frc::SmartDashboard::GetNumber("Right Climber Extend FF Gain", 0.0);
  m_rightClimberExtendCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Right Climber Extend Min Output", 0.0);
  m_rightClimberExtendCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Right Climber Extend Max Output", 0.0);
  m_climbExtendPointR = frc::SmartDashboard::GetNumber("Right Climber Extend Point", 0.0);

  
}

/**
 * Method to initialize PID controllers for the climber
 */

//To be replaced with constructor
void Climber::ClimberPIDInit(){

  //Initialize right climber rotation PID Controller
  m_rightClimberRotatePIDController.SetP(m_rightClimberRotateCoeff.kP);
  std::cout << "right kP: " << m_rightClimberRotateCoeff.kP << "\n";
  m_rightClimberRotatePIDController.SetI(m_rightClimberRotateCoeff.kI);
  m_rightClimberRotatePIDController.SetD(m_rightClimberRotateCoeff.kD);
  m_rightClimberRotatePIDController.SetIZone(m_rightClimberRotateCoeff.kIz);
  m_rightClimberRotatePIDController.SetFF(m_rightClimberRotateCoeff.kFF);
  m_rightClimberRotatePIDController.SetOutputRange(m_rightClimberRotateCoeff.kMinOutput, m_rightClimberRotateCoeff.kMaxOutput);

  // Initialize left climber rotation PID Controller
  m_leftClimberRotatePIDController.SetP(m_leftClimberRotateCoeff.kP);
  std::cout << "left kP: " << m_leftClimberRotateCoeff.kP << "\n";
  m_leftClimberRotatePIDController.SetI(m_leftClimberRotateCoeff.kI);
  m_leftClimberRotatePIDController.SetD(m_leftClimberRotateCoeff.kD);
  m_leftClimberRotatePIDController.SetIZone(m_leftClimberRotateCoeff.kIz);
  m_leftClimberRotatePIDController.SetFF(m_leftClimberRotateCoeff.kFF);
  m_leftClimberRotatePIDController.SetOutputRange(m_leftClimberRotateCoeff.kMinOutput, m_leftClimberRotateCoeff.kMaxOutput);

  // Smart Motion for Rotation
  m_rightClimberRotatePIDController.SetSmartMotionMaxVelocity(kRMaxVel); 
  m_rightClimberRotatePIDController.SetSmartMotionMinOutputVelocity(kRMinVel);
  m_rightClimberRotatePIDController.SetSmartMotionMaxAccel(kRMaxAcc);
  m_rightClimberRotatePIDController.SetSmartMotionAllowedClosedLoopError(kRAllErr);

  m_leftClimberRotatePIDController.SetSmartMotionMaxVelocity(kLMaxVel); 
  m_leftClimberRotatePIDController.SetSmartMotionMinOutputVelocity(kLMinVel);
  m_leftClimberRotatePIDController.SetSmartMotionMaxAccel(kLMaxAcc);
  m_leftClimberRotatePIDController.SetSmartMotionAllowedClosedLoopError(kLAllErr);


  // Set to factory default hardware to prevent unexpected behavior
  m_leftClimberExtender.ConfigFactoryDefault();
  m_rightClimberExtender.ConfigFactoryDefault();

  // Configure Sensor Source for Primary PID on both extend/retract controllers
  m_leftClimberExtender.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  m_rightClimberExtender.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

  // Set inversion type for both climber extend/retract controllers
  m_leftClimberExtender.SetInverted(TalonFXInvertType::CounterClockwise);
  m_rightClimberExtender.SetInverted(TalonFXInvertType::CounterClockwise);

  // Set relevant frame periods to be at least as fast as periodic rate for both extend/retract controllers
  m_leftClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_leftClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  m_rightClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_rightClimberExtender.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  // Set the peak and nominal outputs for left climber extend/retract controller
  m_leftClimberExtender.ConfigNominalOutputForward(0, 10);
  m_leftClimberExtender.ConfigNominalOutputReverse(0, 10);
  m_leftClimberExtender.ConfigPeakOutputForward(m_leftClimberExtendCoeff.kMaxOutput, 10);
  m_leftClimberExtender.ConfigPeakOutputReverse(m_leftClimberExtendCoeff.kMinOutput, 10);

  // Set the peak and nominal outputs for right climber extend/retract controller
  m_rightClimberExtender.ConfigNominalOutputForward(0, 10);
  m_rightClimberExtender.ConfigNominalOutputReverse(0, 10);
  m_rightClimberExtender.ConfigPeakOutputForward(m_rightClimberExtendCoeff.kMaxOutput, 10);
  m_rightClimberExtender.ConfigPeakOutputReverse(m_rightClimberExtendCoeff.kMinOutput, 10);

  // Set Motion Magic gains in slot0 for left climber extend/retract controller
  m_leftClimberExtender.SelectProfileSlot(0, 0);
  m_leftClimberExtender.Config_kF(0, m_leftClimberExtendCoeff.kFF, 10);
  m_leftClimberExtender.Config_kP(0, m_leftClimberExtendCoeff.kP, 10);
  m_leftClimberExtender.Config_kI(0, m_leftClimberExtendCoeff.kI, 10);
  m_leftClimberExtender.Config_kD(0,m_leftClimberExtendCoeff.kD, 10);

  // Set Motion Magic gains in slot0 for right climber extend/retract controller
  m_rightClimberExtender.SelectProfileSlot(0, 0);
  m_rightClimberExtender.Config_kF(0, m_rightClimberExtendCoeff.kFF, 10);
  m_rightClimberExtender.Config_kP(0, m_rightClimberExtendCoeff.kP, 10);
  m_rightClimberExtender.Config_kI(0, m_rightClimberExtendCoeff.kI, 10);
  m_rightClimberExtender.Config_kD(0,m_rightClimberExtendCoeff.kD, 10);

  // Set acceleration and vcruise velocity for the left climber extend/retract controller
  m_leftClimberExtender.ConfigMotionCruiseVelocity(1500, 10);
  m_leftClimberExtender.ConfigMotionAcceleration(1500, 10);

  // Set acceleration and vcruise velocity for the left climber extend/retract controller
  m_rightClimberExtender.ConfigMotionCruiseVelocity(1500, 10);
  m_rightClimberExtender.ConfigMotionAcceleration(1500, 10);

  // Zero the sensor for both controllers
  m_leftClimberExtender.SetSelectedSensorPosition(0, 0, 10);
  m_rightClimberExtender.SetSelectedSensorPosition(0, 0, 10);
}

/**
 * Method for autonomous climber control. Progress the state machine if able.
 * Otherwise, evaluate current state and recover to safe mode if needed.
 */
void Climber::Progress() {

  // Check to see if you can make progress
  if (CanIProgress()) {
    // Move to the next state
    m_phase++;
  }
  else {
    // This is where the climber needs to check IF it needs
    // to reset state. If it does, then recover. Otherwise,
    // stay in the current state
  }
}
/**
 * Method to check if the state machine for climbing can progress
 * 
 * returns:
 *   bool - true if you can progress, false if you cannot
 */
bool Climber::CanIProgress() {

  // Evaluation logic based on current phase
  switch(m_phase) {
    case 0:
    return true;
      break;
    
    case 1: 
      //return abs(m_rightClimberEncoder.GetPosition()) <= (centerR + 2) && abs(m_rightClimberEncoder.GetPosition()) >= (centerR - 2);  
      return true;
      break;

    case 2:
      //return m_rightExtenderServo.Get() == 0.7 && m_rightClimberExtender.GetSelectedSensorPosition() == kMaxRight; 
      return true;
      break;

    case 3:
      //return m_rightExtenderServo.Get() == 0.0 && m_rightClimberExtender.GetSelectedSensorPosition() == kMinRight;
      return true;
      break;

    case 4:
     //return abs(m_leftClimberEncoder.GetPosition()) <= (highL + 2) && abs(m_leftClimberEncoder.GetPosition()) >= (highL - 2);
     return true;
      break;

    case 5:
     return true;
      break;

    case 6:

     return true;
      break;

    case 7:
     // return m_rightClimberExtender.GetSelectedSensorPosition() == phaseSevenRetract 
      //  && m_leftClimberExtender.GetSelectedSensorPosition() == phaseSevenExtend;
      return true;
      break;

    case 8:
    //  return m_leftClimberEncoder.GetPosition() == phaseEightRotate;
    return true;
      break;

    case 9:
     // return m_leftClimberExtender.GetSelectedSensorPosition() == phaseNineRetract;
     return true;
      break;
  }
}

/**
 * Method to kill the climber if needed. "kill" at the moment is 
 * just a state machine reset to the initial state
 */
void Climber::Kill() {
  m_phase = 0;
}

void Climber::Shuffleboard(){ // I can't think of a better place to do this
  if (m_leftExtenderServo.Get() == 0.7) { // Disengaged
    m_leftServoShuffleboard.SetBoolean(false);
  }
  else{
    m_leftServoShuffleboard.SetBoolean(true);
  }
  if(m_rightExtenderServo.Get() == 0.7){ //Disengaged
    m_rightServoShuffleboard.SetBoolean(false);
  }
  else{
    m_rightServoShuffleboard.SetBoolean(true);
  }
}

/**
 * Method to be called every period (default 20ms) to run the current state
 * of the state machine.
 */
void Climber::Run() {
  switch(m_phase) {
    case 0:
    //Default: all motors off, ratchets engaged (should be when servo is zeroed?)
    SetLeftServo(0.0);
    SetRightServo(0.0);
    EngageLeft(0.0);
    EngageRight(0.0);
    RotateRThrottle(0.0);
    RotateLThrottle(0.0);
    break;
    case 1: 
      // Center Left Arm, rotate right arm out of the way (might not be necessary depending on which arm we choose to start w/)

      RotateRight(22.0);

      break;

    case 2:
    //Ratchet disengages, set soft limits for each case??
      // Extend left arm (could possibly merge w/ case 1), driver then drives up to bar 
      RotateLeft(centerL);

      /*
      if (m_rightExtenderServo.Get() == 0.4) {
        EngageRight(0.5); 
      }
      else {
        EngageRight(0.0);
      }
      */
      break;

    case 3:
      EngageLeft(0.5);
      /*
      //Ratchet reengages, Contract right fully
      SetRightServo(0.0);
      if (m_rightExtenderServo.Get() == 0.0) {
        EngageRight(-0.5);
      }
      else {
        EngageRight(0.0);
      }
      */
      break;

    case 4:
      SetLeftServo(0.0);
      
      break;

    case 5:
      // Left contracts
      EngageLeft(-0.5);
      
      break;

    case 6: 
    // Disengage Right ratchet
      EngageLeft(0.0);
      SetRightServo(rightDisengaged);
      
      break;

    case 7: 
    // Extend Right
    EngageRight(0.5);
    


      break;

    case 8: 
    // Rotate Right Arm and Bot
    RotateRight(highR);
    RotateLeft(highL);
      
      break;

    case 9: 
    /* no longer need to rotate?
      //rotate right
      RotateRight(highR);
      */
     // Contract right
     SetRightServo(0.0);
      break;
    
    case 10: 
    EngageRight(-0.5);
    break;
  }

}

void Climber::GetEncoderValues() {
  // Puts Encoder Values
  frc::SmartDashboard::PutNumber("Current Left Climber Rotation Position: ", m_leftClimberEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Current Right Climber Rotation Position: ", m_rightClimberEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Current Left Climber Extension Position: ", m_leftClimberExtender.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Current Right Climber Extension Position: ", m_rightClimberExtender.GetSelectedSensorPosition());
/*
  std::cout << "Left Extension Position: " << m_leftClimberExtender.GetSelectedSensorPosition() << "\n";
  std::cout << "Left Rotation Position: " << m_leftClimberEncoder.GetPosition() << "\n";
  std::cout << "Right Extension Position: " << m_rightClimberExtender.GetSelectedSensorPosition() << "\n";
  std::cout << "Right Rotation Position: " << m_rightClimberEncoder.GetPosition() << "\n";
  */
 
  }

void Climber::InitializeEncoders() {
  m_leftClimberExtender.SetSelectedSensorPosition(0.0);
  m_leftClimberEncoder.SetPosition(0.0);
  m_rightClimberExtender.SetSelectedSensorPosition(0.0);
  m_rightClimberEncoder.SetPosition(0.0);
}

void Climber::InitializeSoftLimits() {
  m_leftClimberExtender.SetInverted(true);
  
  m_leftClimberExtender.ConfigForwardSoftLimitEnable(true);
  m_leftClimberExtender.ConfigReverseSoftLimitEnable(true);
  m_leftClimberExtender.ConfigForwardSoftLimitThreshold(kMaxLeft); 
  m_leftClimberExtender.ConfigReverseSoftLimitEnable(kMinLeft);
  

  m_rightClimberExtender.ConfigForwardSoftLimitEnable(true);
  m_rightClimberExtender.ConfigReverseSoftLimitEnable(true);
  m_rightClimberExtender.ConfigForwardSoftLimitThreshold(kMaxRight); 
  m_rightClimberExtender.ConfigReverseSoftLimitEnable(kMinRight);
  
  
}

void Climber::TestDashInit() {
  // PID Constants for Left Climber Rotation
  frc::SmartDashboard::PutNumber("Left Climber Rotate P Gain", m_leftClimberRotateCoeff.kP);
  frc::SmartDashboard::PutNumber("Left Climber Rotate I Gain", m_leftClimberRotateCoeff.kI);
  frc::SmartDashboard::PutNumber("Left Climber Rotate D Gain", m_leftClimberRotateCoeff.kD);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Max Output", m_leftClimberRotateCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Left Climber Rotate Min Output", m_leftClimberRotateCoeff.kMinOutput);

  // PID Constants for Right Climber Rotation
  frc::SmartDashboard::PutNumber("Right Climber Rotate P Gain", m_rightClimberRotateCoeff.kP);
  frc::SmartDashboard::PutNumber("Right Climber Rotate I Gain", m_rightClimberRotateCoeff.kI);
  frc::SmartDashboard::PutNumber("Right Climber Rotate D Gain", m_rightClimberRotateCoeff.kD);
  frc::SmartDashboard::PutNumber("Right Climber Rotate Max Output", m_rightClimberRotateCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Right Climber Rotate Min Output", m_rightClimberRotateCoeff.kMinOutput);

  // Smart Motion
  frc::SmartDashboard::PutNumber("SM: Left Climber Max Velocity", m_leftClimberRotatePIDController.GetSmartMotionMaxVelocity());
  frc::SmartDashboard::PutNumber("SM: Left Climber Min Velocity", m_leftClimberRotatePIDController.GetSmartMotionMinOutputVelocity());
  frc::SmartDashboard::PutNumber("SM: Left Climber Max Accel", m_leftClimberRotatePIDController.GetSmartMotionMaxAccel());
  frc::SmartDashboard::PutNumber("SM: Left Climber Error Allowed", m_leftClimberRotatePIDController.GetSmartMotionAllowedClosedLoopError());

  frc::SmartDashboard::PutNumber("SM: Right Climber Max Velocity", m_rightClimberRotatePIDController.GetSmartMotionMaxVelocity());
  frc::SmartDashboard::PutNumber("SM: Right Climber Min Velocity", m_rightClimberRotatePIDController.GetSmartMotionMinOutputVelocity());
  frc::SmartDashboard::PutNumber("SM: Right Climber Max Accel", m_rightClimberRotatePIDController.GetSmartMotionMaxAccel());
  frc::SmartDashboard::PutNumber("SM: Right Climber Error Allowed", m_rightClimberRotatePIDController.GetSmartMotionAllowedClosedLoopError());

  // Rotation Positions
  frc::SmartDashboard::PutNumber("Right Climber Rotation Point", m_rotationR);
  frc::SmartDashboard::PutNumber("Left Climber Rotation Point", m_rotationL);


}

void Climber::TestReadDash() {

  // Read climber rotation PID constants
  m_rightClimberRotateCoeff.kP  = frc::SmartDashboard::GetNumber("Right Climber Rotate P Gain", 0.0);
  m_rightClimberRotateCoeff.kI   = frc::SmartDashboard::GetNumber("Right Climber Rotate I Gain", 0.0);
  m_rightClimberRotateCoeff.kD   = frc::SmartDashboard::GetNumber("Right Climber Rotate D Gain", 0.0);
  m_rightClimberRotateCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Right Climber Rotate Min Output", 0.0);
  m_rightClimberRotateCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Right Climber Rotate Max Output", 0.0);

  m_leftClimberRotateCoeff.kP   = frc::SmartDashboard::GetNumber("Left Climber Rotate P Gain", 0.0);
  m_leftClimberRotateCoeff.kI   = frc::SmartDashboard::GetNumber("Left Climber Rotate I Gain", 0.0);
  m_leftClimberRotateCoeff.kD   = frc::SmartDashboard::GetNumber("Left Climber Rotate D Gain", 0.0);
  m_leftClimberRotateCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Left Climber Rotate Min Output", 0.0);
  m_leftClimberRotateCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Left Climber Rotate Max Output", 0.0);

  // Smart Motion
  kLMaxVel = frc::SmartDashboard::GetNumber("SM: Left Climber Max Velocity", m_leftClimberRotatePIDController.GetSmartMotionMaxVelocity());
  kLMinVel = frc::SmartDashboard::GetNumber("SM: Left Climber Min Velocity", m_leftClimberRotatePIDController.GetSmartMotionMinOutputVelocity());
  kLMaxAcc = frc::SmartDashboard::GetNumber("SM: Left Climber Max Accel", m_leftClimberRotatePIDController.GetSmartMotionMaxAccel());
  kLAllErr = frc::SmartDashboard::GetNumber("SM: Left Climber Error Allowed", m_leftClimberRotatePIDController.GetSmartMotionAllowedClosedLoopError());

  kRMaxVel = frc::SmartDashboard::GetNumber("SM: Right Climber Max Velocity", m_rightClimberRotatePIDController.GetSmartMotionMaxVelocity());
  kRMinVel = frc::SmartDashboard::GetNumber("SM: Right Climber Min Velocity", m_rightClimberRotatePIDController.GetSmartMotionMinOutputVelocity());
  kRMaxAcc = frc::SmartDashboard::GetNumber("SM: Right Climber Max Accel", m_rightClimberRotatePIDController.GetSmartMotionMaxAccel());
  kRAllErr = frc::SmartDashboard::GetNumber("SM: Right Climber Error Allowed", m_rightClimberRotatePIDController.GetSmartMotionAllowedClosedLoopError());

  m_rotationR = frc::SmartDashboard::GetNumber("Right Climber Rotation Point", 0.0);
  m_rotationL = frc::SmartDashboard::GetNumber("Left Climber Rotation Point", 0.0);

}

void Climber::TestL() {
  //std::cout << "Left Rotation Point: " << m_rotationL << "\n";
  RotateLeft(m_rotationL);
}


void Climber::TestR() {
  //std::cout << "Right Rotation Point: " << m_rotationR << "\n";
  //std::cout << "right p: " << m_rightClimberRotateCoeff.kP << "\n";
  RotateRight(m_rotationR);
}

void Climber::RotateRThrottle(double throttle) {
  m_rightClimberRotationNeo.Set(throttle); 
}

void Climber::RotateLThrottle(double throttle) {
  m_leftClimberRotationNeo.Set(throttle); 
}

//automatically sets climber to a certain stage (e.g. case 1, 2, 3)
void Climber::SetPhase(int phase) {
  m_phase = phase; 
}

int Climber::GetPhase() {
  return m_phase;
}

void Climber::SetLeftServo(double position) {
  m_leftExtenderServo.Set(position);
}

void Climber::SetRightServo(double position) {
  m_rightExtenderServo.Set(position);
}


