// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit()
{
  m_joy_op = new frc::Joystick(0);
  
  m_drive = new DriveBase(m_joy_op);
  m_climber = new Climber();
  m_hood = new Hood();
  m_intake = new Intake();
  m_shooter = new Shooter();
  m_indexer = new Indexer();
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{

  if (m_joy_op->GetRawButton(1)) {
    m_indexer->SetState(IndexerState::INTAKE);
  }
  else if (m_joy_op->GetRawButton(2)) { // B Intake
    m_intake->setState(IntakeState::GO);
    m_indexer->SetState(IndexerState::INTAKE);
  }
  else if (m_joy_op->GetRawButton(3)) { // X Far Shoot
    m_shooter->setState(ShooterState::SHOOT);
    m_hood->setState(HoodState::DOWN);
    m_indexer->SetState(IndexerState::SHOOT);
  }
  else if (m_joy_op->GetRawButton(4)) { // Y Close Shoot
    m_shooter->setState(ShooterState::SHOOT);
    m_hood->setState(HoodState::UP);
    m_indexer->SetState(IndexerState::SHOOT);
  }
  else if (m_joy_op->GetRawButton(5)) { // Left bumper #1 Climb
    m_climber->current_state = States::DOWN_CLIMB;
  }
  else if (m_joy_op->GetRawButton(6)) { // Right bumper #2 Climb
    m_climber->current_state = States::UP_CLIMB;
  }
  else if (m_joy_op->GetRawButton(7)) { // Back #4 Climb
    m_climber->current_state = States::ARM_REVERSE;
  }
  else if (m_joy_op->GetRawButton(8)) { // Start #3 Climb
    m_climber->current_state = States::ARM_FORWARD;
  }
  else if (m_joy_op->GetRawButton(9)) {
  
  }
  else if (m_joy_op->GetRawButton(10)) {
  
  } else if (m_joy_op->GetPOV() == 0) {
    m_indexer->SetState(IndexerState::MANUALTOP);
  } else if (m_joy_op->GetPOV() == 180) {
    m_indexer->SetState(IndexerState::MANUALBOTTOM);
  } else if (m_joy_op->GetPOV() == 270) {
    m_indexer->SetState(IndexerState::MANUALBOTH);
  } else {
    m_intake->setState(IntakeState::WAITING);
    m_indexer->SetState(IndexerState::INTAKE);
    m_shooter->setState(ShooterState::WAITING);
  }

  m_climber->climberStateMachine();
  m_hood->HoodStateMachine();
  m_intake->IntakeStateMachine();
  m_shooter->ShooterStateMachine();
  m_indexer->IndexerStateMachine();

  m_drive->Controller();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
