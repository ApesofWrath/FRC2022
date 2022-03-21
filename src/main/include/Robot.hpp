// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/Compressor.h>

#include <AHRS.h>

#include "Drive/DriveBase.hpp"
#include "Climber.hpp"
#include "Hood.hpp"
#include "Intake.hpp"
#include "Shooter.hpp"
#include "Indexer.hpp"
#include "RobotContainer.hpp"
#include "Auton/Auton.hpp"
#include "cameraserver/CameraServer.h"

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
  Climber *m_climber;
  std::shared_ptr<Shooter> m_shooter;
  std::shared_ptr<Indexer> m_indexer;
  std::shared_ptr<Hood> m_hood;
  std::shared_ptr<Intake> m_intake;
  std::shared_ptr<frc::Compressor> m_compressor;
  frc::Joystick *m_joy_op;

  WPI_TalonFX *m_talon;
  AutonDrive *m_AutonDrive;
  RobotContainer *m_Container;
  AHRS* m_AHRS;

  frc2::Command* m_AutonomousCommand = nullptr;
};
