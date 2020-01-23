/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/Compressor.h>
#include <frc/Joystick.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/ADXRS450_Gyro.h> 
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

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
  frc::Compressor* c = new frc::Compressor(0);
  TalonSRX* intake_motor;
  frc::DoubleSolenoid* intake;
  frc::PowerDistributionPanel PDP = frc::PowerDistributionPanel(0);
  frc::Joystick stick = frc::Joystick(0);
  const double WHEEL_DIAMETER = 0.1524;
  const double GEARING = 10.42;
  const int PIDIDX = 0;
  double encoderConstant = (1 / GEARING) * WHEEL_DIAMETER * M_PI;

  rev::CANSparkMax m_leftLeadMotor{16, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{14, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{15, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{13, rev::CANSparkMax::MotorType::kBrushless};
  
  frc::SpeedControllerGroup m_leftMotors{m_leftLeadMotor, m_leftFollowMotor};

  frc::SpeedControllerGroup m_rightMotors{m_rightLeadMotor, m_rightFollowMotor};

  frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};

  rev::CANEncoder m_leftEncoder = m_leftLeadMotor.GetEncoder();
  rev::CANEncoder m_rightEncoder = m_rightLeadMotor.GetEncoder();

  frc::ADXRS450_Gyro m_gyro = frc::ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);
  
  nt::NetworkTableEntry autoSpeedEntry =
    nt::NetworkTableInstance::GetDefault().GetEntry("/robot/autospeed");
  nt::NetworkTableEntry telemetryEntry =
    nt::NetworkTableInstance::GetDefault().GetEntry("/robot/telemetry");
  nt::NetworkTableEntry rotateEntry =
    nt::NetworkTableInstance::GetDefault().GetEntry("/robot/rotate");
  
  double priorAutospeed = 0;
  double numberArray[10] = {0};
};
