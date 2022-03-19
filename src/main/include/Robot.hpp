// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Drive/DriveBase.hpp"
#include "cameraserver/CameraServer.h"

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
  TalonFX *m_talon1;
  TalonFX *m_talon2;
  TalonFX *m_talon3;
  TalonFX *m_talon4;
  AHRS *ahrs;

  double max_rpm = 0;
  double max_yaw = 0;
};
