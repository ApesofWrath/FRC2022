// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit()
{
  m_joy_op = new frc::Joystick(0);

  m_drive = new DriveBase(m_joy_op);
  m_climber = new Climber();
  m_shooter = std::make_shared<Shooter>();
  m_hood = std::make_shared<Hood>();
  m_intake = std::make_shared<Intake>();
  m_indexer = std::make_shared<Indexer>(m_shooter, m_intake);
  m_compressor = std::make_shared<frc::Compressor>(61, frc::PneumaticsModuleType::CTREPCM);

  frc::CameraServer::StartAutomaticCapture();
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Field View", 320, 190);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    m_compressor->EnableDigital();
}
void Robot::TeleopPeriodic()
{
  if (frc::DriverStation::GetMatchTime() > 33)
  {
    if(m_joy_op->GetRawButton(2) && m_joy_op->GetRawButton(5) && m_joy_op->GetRawButton(7)) {
    m_climb_mode = true;
    }
    m_climb_time = true;
  }
  

  if (!m_climb_mode)
  { 
    if (m_joy_op->GetRawButton(1))
    { // A Manually Triggered Automatic Indexing
      m_indexer->SetState(IndexerState::INTAKE);
      m_intake->setState(IntakeState::INDEXING);
    }
    else if (m_joy_op->GetRawButton(2))
    { // B Intake
      m_intake->setState(IntakeState::GO);
      m_indexer->SetState(IndexerState::INTAKE);
    }
    else if (m_joy_op->GetRawButton(3))
    { // X Far Shoot
      m_hood->setState(HoodState::DOWN);
    }
    else if (m_joy_op->GetRawButton(4))
    { // Y Close Shoot
      m_hood->setState(HoodState::UP);
    }
    else if (m_joy_op->GetRawButton(5)) {
      m_shooter->setState(ShooterState::WAITING);
    }
    else if (m_joy_op->GetRawButton(6)) {
      m_shooter->setState(ShooterState::SHOOT);
      m_indexer->SetState(IndexerState::SHOOT);
      
    }
    else if (m_joy_op->GetRawButton(7)) {
      
    }
    else if (m_joy_op->GetRawButton(8)) {
      
    }
    else if (m_joy_op->GetRawButton(9))
    {
    }
    else if (m_joy_op->GetRawButton(10))
    {
    }
    else if (m_joy_op->GetPOV() == 0)
    {
      m_indexer->SetState(IndexerState::MANUALTOP);
    }
    else if (m_joy_op->GetPOV() == 180)
    {
      m_indexer->SetState(IndexerState::MANUALBOTTOM);
    }
    else if (m_joy_op->GetPOV() == 270)
    {
      m_indexer->SetState(IndexerState::MANUALBOTH);
    }
    else if(m_joy_op->GetRawAxis(3) > 0.5) { // Trigger Intake, positive right negative left 
    }
    else
    {
      if (m_intake->getState() != IntakeState::INDEXING)
            m_intake->setState(IntakeState::STOP);
      m_indexer->SetState(IndexerState::WAITING);
      m_shooter->setState(ShooterState::STOP);
    }
  } else {
    if (m_joy_op->GetRawButton(5))
    { // Left bumper #1 Climb
      m_climber->current_state = States::DOWN_CLIMB;
    }
    else if (m_joy_op->GetRawButton(6))
    { // Right bumper #2 Climb
      m_climber->current_state = States::UP_CLIMB;
    }
    else if (m_joy_op->GetRawButton(7))
    { // Back #4 Climb
      m_climber->current_state = States::ARM_REVERSE;
    }
    else if (m_joy_op->GetRawButton(8))
    { // Start #3 Climb
      m_climber->current_state = States::ARM_FORWARD;
    }
  }

  m_climber->climberStateMachine();
  m_hood->HoodStateMachine();
  m_intake->IntakeStateMachine();
  m_shooter->ShooterStateMachine();
  m_indexer->IndexerStateMachine();

  m_drive->Controller();
}

void Robot::DisabledInit() {
  m_compressor->Disable();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
