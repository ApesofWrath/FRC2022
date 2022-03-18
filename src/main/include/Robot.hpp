// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>

#include "Drive/DriveBase.hpp"
#include "Climber.hpp"
#include "Hood.hpp"
#include "Intake.hpp"
#include "Shooter.hpp"
#include "Indexer.hpp"

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
  Climber *m_climber;
  Hood *m_hood;
  Intake *m_intake;
  Shooter *m_shooter;
  Indexer *m_indexer;
  frc::Joystick *m_joy_op;
};
