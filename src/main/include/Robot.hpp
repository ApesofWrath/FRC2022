// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <memory>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/Compressor.h>

#include "Shooter.hpp"
#include "Hood.hpp"
#include "Intake.hpp"

#include "Drive/DriveBase.hpp"
#include "Shooter.hpp"

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
  std::shared_ptr<Shooter> m_shooter;

  std::shared_ptr<Hood> m_hood;
  std::shared_ptr<Intake> m_intake;
  std::shared_ptr<frc::Compressor> m_compressor;

};
