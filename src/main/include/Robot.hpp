// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>

#include <AHRS.h>

#include "Drive/DriveBase.hpp"
#include "RobotContainer.hpp"
#include "Auton/Auton.hpp"

#include <frc2/command/Command.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

private:
  DriveBase *m_drive;
  frc::Joystick *m_joy_op;
  WPI_TalonFX *m_talon;
  AutonDrive *m_AutonDrive;
  RobotContainer *m_Container;
  AHRS* m_AHRS;

  frc2::Command* m_AutonomousCommand = nullptr;
};
