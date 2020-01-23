/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  m_drive.SetDeadband(0);
  
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  nt::NetworkTableInstance::GetDefault().SetUpdateRate(0.010);
  intake_motor = new TalonSRX(11);
  intake = new frc::DoubleSolenoid(4, 5);
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
}

void Robot::AutonomousPeriodic()
{
  double now = frc::Timer::GetFPGATimestamp();
  double leftPosition = m_leftEncoder.GetPosition() * encoderConstant;
  double rightPosition = m_rightEncoder.GetPosition() * encoderConstant;
  double leftRate = m_leftEncoder.GetVelocity() * encoderConstant / 60;
  double rightRate = m_rightEncoder.GetVelocity() * encoderConstant / 60;
  double leftMotorVolts = m_leftLeadMotor.GetBusVoltage() * m_leftLeadMotor.GetAppliedOutput();
  double rightMotorVolts = m_rightLeadMotor.GetBusVoltage() * m_rightLeadMotor.GetAppliedOutput();
  double battery = PDP.GetVoltage();
  double autospeed = autoSpeedEntry.GetDouble(0);
  priorAutospeed = autospeed;

  m_drive.TankDrive(
    (rotateEntry.GetBoolean(false) ? -1 : 1) * autospeed, autospeed,
    false
  );
  numberArray[0] = now;
  numberArray[1] = battery;
  numberArray[2] = autospeed;
  numberArray[3] = leftMotorVolts;
  numberArray[4] = rightMotorVolts;
  numberArray[5] = leftPosition;
  numberArray[6] = rightPosition;
  numberArray[7] = leftRate;
  numberArray[8] = rightRate;
  numberArray[9] = -1 * m_gyro.GetAngle() * (M_PI / 180);

  telemetryEntry.SetDoubleArray(numberArray);
}

void Robot::TeleopInit() {c->Start();}

void Robot::TeleopPeriodic()
{
  m_drive.ArcadeDrive(-stick.GetRawAxis(1), stick.GetRawAxis(4) / 2);
  intake_motor->Set(ControlMode::PercentOutput, stick.GetRawAxis(3) - stick.GetRawAxis(2));
  intake->Set(stick.GetRawButton(1) ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
