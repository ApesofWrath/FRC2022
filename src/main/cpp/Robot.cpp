// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit() {
  m_joy_op = new frc::Joystick(0);
  m_joy_drive = new frc::Joystick(1);
  
  m_drive = new DriveBase(m_joy_op, m_AHRS);
  m_climber = new Climber();
  m_shooter = std::make_shared<Shooter>();
  m_hood = std::make_shared<Hood>();
  m_intake = std::make_shared<Intake>();
  m_indexer = std::make_shared<Indexer>(m_shooter, m_intake);
  m_compressor = std::make_shared<frc::Compressor>(61, frc::PneumaticsModuleType::CTREPCM);

  frc::CameraServer::StartAutomaticCapture();
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Field View", 320, 190);

  //why is there a talon here???????
  m_talon = new WPI_TalonFX(0);

  m_AHRS = new AHRS(frc::SerialPort::kMXP);

  m_AutonDrive = new AutonDrive(10,12,11,13, m_AHRS);
  m_Container = new RobotContainer(m_AutonDrive);

  m_Container->InitAutoChoices();

  frc::CameraServer::StartAutomaticCapture();
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Field View", 320, 190);
}

  frc::SmartDashboard::PutData("Auto Modes", &(m_Container->m_chooser));
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

}

void Robot::AutonomousInit() {
  m_drive->SetBrake();
  m_compressor->EnableDigital();
  m_Container->m_autoSelected = m_Container->m_chooser.GetSelected();

  if (m_AutonomousCommand != nullptr) {
    m_AutonomousCommand->Cancel();
    m_AutonomousCommand = nullptr;
  }
  m_AutonDrive->resetOdometry(frc::Pose2d( 0_m, 0_m, frc::Rotation2d(0_deg)));  

  m_AutonomousCommand = m_Container->GetAutonomousCommand();

  if (m_AutonomousCommand != nullptr) {
    m_AutonomousCommand->Schedule();
  }

}
void Robot::AutonomousPeriodic() {

  frc::Pose2d pose = m_AutonDrive->getPose();

  frc::SmartDashboard::PutNumber("heading", pose.Rotation().Degrees().value());
  frc::SmartDashboard::PutNumber("translation x", pose.Translation().X().value());
  frc::SmartDashboard::PutNumber("translation y", pose.Translation().Y().value());

  // ??
  m_climber->climberStateMachine();
  m_hood->HoodStateMachine();
  m_intake->IntakeStateMachine();
  m_shooter->ShooterStateMachine();
  m_indexer->IndexerStateMachine();
  
}

void Robot::TeleopInit() {
  frc2::CommandScheduler::GetInstance().Disable();
  if (m_AutonomousCommand != nullptr) {
    m_AutonomousCommand->Cancel();
    m_AutonomousCommand = nullptr;
  }
  m_drive->SetCoast();


}
void Robot::TeleopPeriodic() {
  m_drive->Controller();
}

void Robot::DisabledInit() {
  m_drive->SetBrake();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
