// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit()
{
  m_joy_op = new frc::Joystick(0);
  m_joy_drive = new frc::Joystick(1);

  m_drive = new DriveBase(m_joy_drive);
  m_shooter = std::make_shared<Shooter>();
  m_hood = std::make_shared<Hood>();
  m_intake = std::make_shared<Intake>();
  m_indexer = std::make_shared<Indexer>(m_shooter, m_intake);
  m_compressor = std::make_shared<frc::Compressor>(61, frc::PneumaticsModuleType::CTREPCM);

  m_ahrs = new AHRS(frc::SerialPort::kMXP);

  m_autondrive = new AutonDrive(10, 12, 11, 13, m_ahrs);
  m_container = new RobotContainer(m_autondrive);

  m_container->InitAutoChoices();

  frc::CameraServer::StartAutomaticCapture();
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Field View", 320, 190);

  frc::SmartDashboard::PutData("Auto Modes", &(m_container->m_chooser));
}
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
  m_drive->SetBrakeNeutral();
  m_compressor->EnableDigital();
  m_container->m_autoSelected = m_container->m_chooser.GetSelected();

  if (m_AutonomousCommand != nullptr) {
    m_AutonomousCommand->Cancel();
    m_AutonomousCommand = nullptr;
  }
  m_autondrive->resetOdometry(frc::Pose2d( 0_m, 0_m, frc::Rotation2d(0_deg)));  

  m_AutonomousCommand = m_container->GetAutonomousCommand();

  if (m_AutonomousCommand != nullptr) {
    m_AutonomousCommand->Schedule();
  }

}
void Robot::AutonomousPeriodic() {
  frc::Pose2d pose = m_autondrive->getPose();

  m_climber->climberStateMachine();
  m_hood->HoodStateMachine();
  m_intake->IntakeStateMachine();
  m_shooter->ShooterStateMachine();
  m_indexer->IndexerStateMachine();
}

void Robot::TeleopInit()
{
  frc2::CommandScheduler::GetInstance().Disable();
  if (m_AutonomousCommand != nullptr) {
    m_AutonomousCommand->Cancel();
    m_AutonomousCommand = nullptr;
  }
  m_drive->SetCoastNeutral();


  m_compressor->EnableDigital();
  m_climber->Zero();
  m_climber->current_state = States::ARM_FORWARD;
}
void Robot::TeleopPeriodic()
{
  frc::SmartDashboard::PutNumber("joy pov", m_joy_op->GetPOV());
  if (false) // frc::DriverStation::GetMatchTime() > 33)
  {
    if (m_joy_op->GetRawButton(2) && m_joy_op->GetRawButton(5) && m_joy_op->GetRawButton(7))
    {
      m_climb_mode = true;
    }
    m_climb_time = true;
  }

  /*
  //climber testing stuff
  if(m_joy_op->GetRawButton(5)) {
    m_climber->current_state = States::UP_CLIMB;
  } else if(m_joy_op->GetRawButton(6)) {

  } else if(m_joy_op->GetRawButton(7)) {
    m_climber->current_state = States::DOWN_CLIMB;
  }
  */

  if (!m_climb_mode)
  {
    // Indexer
    if (m_joy_op->GetRawButton(1))
    {
      m_indexer->SetState(IndexerState::INTAKE);
    }
    else if (m_joy_op->GetRawButton(2))
    {
      m_indexer->SetState(IndexerState::INTAKE);
    }
    else if (m_joy_op->GetRawButton(6))
    {
      m_indexer->SetState(IndexerState::SHOOTERCHECK);
    }
    else if (m_joy_op->GetPOV() == 0)
    {
      m_indexer->SetState(IndexerState::MANUALTOP);
    }
    else if (m_joy_op->GetPOV() == 90)
    {
      m_indexer->SetState(IndexerState::MANUALREVERSETOP);
    }
    else if (m_joy_op->GetPOV() == 180)
    {
      m_indexer->SetState(IndexerState::MANUALBOTTOM);
    }
    else if (m_joy_op->GetPOV() == 270)
    {
      m_indexer->SetState(IndexerState::MANUALBOTH);
    }
    else
    {
      m_indexer->SetState(IndexerState::WAITING);
    }

    // Intake
    if (m_joy_op->GetRawButton(1))
    { // A Manually Triggered Automatic Indexing
      m_intake->setState(IntakeState::INDEXING);
    }
    else if (m_joy_op->GetRawButton(2))
    { // B Intake
      m_intake->setState(IntakeState::GO);
    }
    else if (m_intake->getState() != IntakeState::INDEXING)
    {
      m_intake->setState(IntakeState::STOP);
    }

    // Shooter
    if (m_joy_op->GetRawButton(3) || m_joy_op->GetRawButton(4))
    {
      m_shooter->setState(ShooterState::SPOOLING);
    }
    else if (m_joy_op->GetRawButton(5))
    {
      m_shooter->setState(ShooterState::WAITING);
    }
    else if (m_joy_op->GetRawButton(6))
    {
      m_shooter->setState(ShooterState::SHOOT);
    }
    else if (m_shooter->get_state() != ShooterState::SPOOLING)
    {
      m_shooter->setState(ShooterState::STOP);
    }

    // Hood
    if (m_joy_op->GetRawButton(3))
    { // X Far Shoot
      m_hood->setState(HoodState::DOWN);
    }
    else if (m_joy_op->GetRawButton(4))
    { // Y Close Shoot
      m_hood->setState(HoodState::UP);
    }
  }
  else
  {
    /*
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
    */
  }

  m_climber->climberStateMachine();

  m_hood->HoodStateMachine();
  m_intake->IntakeStateMachine();
  m_shooter->ShooterStateMachine();
  m_indexer->IndexerStateMachine();

  m_drive->Controller();
}

void Robot::DisabledInit()
{
  m_compressor->Disable();
  // m_climber->CoastElevator();
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
