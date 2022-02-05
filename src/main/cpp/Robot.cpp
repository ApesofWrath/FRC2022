// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit() {
  m_joy_op = new frc::Joystick(0);
  m_drive = new DriveBase(m_joy_op, m_AHRS);
  m_talon = new WPI_TalonFX(0);

  m_AHRS = new AHRS(frc::SerialPort::kMXP);

  m_AutonDrive = new AutonDrive(10,12,11,13, m_AHRS);
  m_Container = new RobotContainer(m_AutonDrive);

  m_Container->InitAutoChoices();

  frc::SmartDashboard::PutData("Auto Modes", &(m_Container->m_chooser));
}
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

}

void Robot::AutonomousInit() {

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

  
}

void Robot::TeleopInit() {
  frc2::CommandScheduler::GetInstance().Disable();
  if (m_AutonomousCommand != nullptr) {
    m_AutonomousCommand->Cancel();
    m_AutonomousCommand = nullptr;
  }

}
void Robot::TeleopPeriodic() {
  m_drive->Controller();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
