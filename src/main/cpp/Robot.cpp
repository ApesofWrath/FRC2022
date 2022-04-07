// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit()
{
  m_joy_op = new frc::Joystick(0);
  m_joy_drive = new frc::Joystick(1);

  // m_climber = new Climber();
  m_shooter = std::make_shared<Shooter>();
  m_hood = std::make_shared<Hood>();
  m_intake = std::make_shared<Intake>();
  m_indexer = std::make_shared<Indexer>(m_shooter, m_intake);
  m_compressor = std::make_shared<frc::Compressor>(61, frc::PneumaticsModuleType::CTREPCM);

  m_ahrs = new AHRS(frc::SerialPort::kMXP);

  m_autondrive = new AutonDrive(10, 12, 11, 13, m_ahrs);
  m_container = new RobotContainer(m_autondrive, m_intake, m_indexer, m_shooter, m_hood);
  m_drive = new DriveBase(m_joy_drive, m_ahrs);

  m_container->InitAutoChoices();

  frc::CameraServer::StartAutomaticCapture();
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Field View", 160, 90);

  frc::SmartDashboard::PutData("Auto Modes", &(m_container->m_chooser));
}
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit()
{
  std::cout << "1";
  m_drive->SetBrakeNeutral();
  m_compressor->EnableDigital();
  m_container->m_autoSelected = m_container->m_chooser.GetSelected();
  std::cout << "2";

  if (m_AutonomousCommand != nullptr)
  {
    m_AutonomousCommand->Cancel();
    m_AutonomousCommand = nullptr;
  }
  m_autondrive->configure();
  m_autondrive->resetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
  std::cout << "3";

  m_AutonomousCommand = m_container->GetAutonomousCommand();

  if (m_AutonomousCommand != nullptr)
  {
    m_AutonomousCommand->Schedule();
  }
  std::cout << "4";
}
void Robot::AutonomousPeriodic()
{
  frc::Pose2d pose = m_autondrive->getPose();

  // m_climber->climberStateMachine();
  m_hood->HoodStateMachine();
  m_intake->IntakeStateMachine();
  m_shooter->ShooterStateMachine();
  m_indexer->IndexerStateMachine();
  frc::SmartDashboard::PutNumber("x", m_autondrive->getPose().X().value());
  frc::SmartDashboard::PutNumber("y", m_autondrive->getPose().Y().value());
}

void Robot::TeleopInit()
{
  frc2::CommandScheduler::GetInstance().Disable();
  if (m_AutonomousCommand != nullptr)
  {
    m_AutonomousCommand->Cancel();
    m_AutonomousCommand = nullptr;
  }
  m_drive->UpdateConfigs();
  m_drive->SetCoastNeutral();

  m_compressor->EnableDigital();
  /*
  m_climber->Zero();
  m_climber->current_state = States::ARM_FORWARD;
  */
  m_indexer->SetState(IndexerState::INIT);
  m_shooter->setState(ShooterState::INIT);
  m_hood->setState(HoodState::INIT);
}
void Robot::TeleopPeriodic()
{
  ProcessButtons();
 
  m_joy_op->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
  m_joy_op->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);

  // frc::SmartDashboard::PutNumber("joy pov", m_joy_op->GetPOV());
  if (/*frc::DriverStation::GetMatchTime() > 33 || */ (m_joy_op->GetRawButton(SPOOLING) && m_joy_op->GetRawButton(INTAKE_OUT_1) && m_joy_op->GetRawButton(7))) // frc::DriverStation::GetMatchTime() > 33)
  {
    m_climb_mode = true;
  }

  if (!m_climb_mode)
  {
    // Indexer
    if (m_joy_op->GetRawButton(ControllerButtons::INDEXING))
    {
      m_indexer->SetState(IndexerState::INTAKE);
    }
    else if (m_joy_op->GetRawButton(ControllerButtons::INTAKE_OUT_1) || m_joy_op->GetRawButton(ControllerButtons::INTAKE_OUT_2))
    {
      m_indexer->SetState(IndexerState::INTAKE);
    }
    else if ((m_joy_op->GetRawAxis(2) > 0.6 && m_indexer->GetState() != IndexerState::SHOOT) || (m_joy_op->GetRawAxis(3) > 0.6 && m_indexer->GetState() != IndexerState::SHOOT))
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
    else if (m_indexer->GetState() != IndexerState::SHOOT)
    {
      m_indexer->SetState(IndexerState::WAITING);
    }

    // Intake
    if (m_joy_op->GetRawButton(ControllerButtons::INDEXING))
    { // A Manually Triggered Automatic Indexing
      m_intake->setState(IntakeState::INDEXING);
    }
    else if (m_joy_op->GetRawButton(ControllerButtons::INTAKE_OUT_1) || m_joy_op->GetRawButton(ControllerButtons::INTAKE_OUT_2))
    { // B Intake
      m_intake->setState(IntakeState::GO);
      m_joy_op->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.4);
      m_joy_op->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.4);
    } else if (m_joy_op->GetRawButton(4)) {
      m_intake->setState(IntakeState::ONLY_OUT);
    } else if (m_intake->getState() != IntakeState::INDEXING)
    {
      m_intake->setState(IntakeState::STOP);
    }

    // Shooter
    if (spooling_button)
    {
      m_shooter->setState(ShooterState::SPOOLING);
    }
    else if (hub_shooting_trigger)
    {
      m_shooter->setState(ShooterState::SHOOT_HUB);
    }
    else if (launchpad_shooting_trigger)
    {
      m_shooter->setState(ShooterState::SHOOT_LAUNCHPAD);
    }
    else if (manual_reverse_top_pov)
    {
      m_shooter->setState(ShooterState::REVERSE);
    }
    else if (m_shooter->get_state() != ShooterState::SPOOLING)
    {
      m_shooter->setState(ShooterState::STOP);
    }

    // Hood
    if (hub_shooting_trigger)
    { // X Far Shoot
      m_hood->setState(HoodState::DOWN);
      m_joy_op->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
      m_joy_op->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
    }
    else if (launchpad_shooting_trigger)
    { // Y Close Shoot
      m_hood->setState(HoodState::UP);
      m_joy_op->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
      m_joy_op->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
    }

    /*
    if (m_joy_drive->GetRawButton(1))
    { // Left bumper #1 Climb
      m_climber->current_state = States::DOWN_CLIMB;
    }
    else if (m_joy_drive->GetRawButton(3))
    { // Right bumper #2 Climb
      m_climber->current_state = States::UP_CLIMB;
    } else if(m_joy_drive->GetRawButton(2)){
      m_climber->current_state = States::ARM_REVERSE;
    } else if(m_joy_drive->GetRawButton(4)) {
      m_climber->current_state = States::ARM_FORWARD;
    }
    */
  }

  // if (m_joy_drive->GetRawButton(5)) {
  //   m_climber->climber_talon1->Set(ControlMode::PercentOutput, 0.1);
  // } else {
  //   m_climber->climber_talon1->Set(ControlMode::PercentOutput, 0.0);
  // }

  if (m_joy_drive->GetRawButton(ControllerButtons::INTAKE_OUT_2))
  {
    m_drive->enableSlowMode();
  }
  else
  {
    m_drive->disableSlowMode();
  }

  // m_climber->climberStateMachine();

  m_hood->HoodStateMachine();
  m_intake->IntakeStateMachine();
  m_shooter->ShooterStateMachine();
  m_indexer->IndexerStateMachine();

  m_drive->Controller();
}

void Robot::DisabledInit()
{
  m_compressor->Disable();
  m_drive->SetBrakeNeutral();
  frc::SmartDashboard::PutData("Auto Modes💀", &(m_container->m_chooser));
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::ProcessButtons()
{
  // this.SetState(HandState::THUMBS_UP);
  index_button = m_joy_op->GetRawButton(ControllerButtons::INDEXING);
  spooling_button = m_joy_op->GetRawButton(ControllerButtons::SPOOLING);
  hub_shooting_trigger = m_joy_op->GetRawAxis(ControllerAxes::HUB_SHOOTING);
  launchpad_shooting_trigger = m_joy_op->GetRawAxis(ControllerAxes::LAUNCHPAD_SHOOTING);
  manual_reverse_top_pov = m_joy_op->GetPOV() == ControllerPOVs::MANUAL_REVERSE_TOP;
  manual_top_pov = m_joy_op->GetPOV() == ControllerPOVs::MANUAL_TOP;
  manual_bottom_pov = m_joy_op->GetPOV() == ControllerPOVs::MANUAL_BOTTOM;
  manual_both_pov = m_joy_op->GetPOV() == ControllerPOVs::MANUAL_BOTH;

  (m_joy_op->GetRawButton(ControllerButtons::INTAKE_OUT_1) || m_joy_op->GetRawButton(ControllerButtons::INTAKE_OUT_2))
      ? intake_button = true
      : intake_button = false;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
