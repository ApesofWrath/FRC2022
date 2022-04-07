// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define LOG_V(var) frc::SmartDashboard::PutNumber(#var , var)

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/Compressor.h>

#include "Drive/DriveBase.hpp"
#include "Climber.hpp"
#include "Hood.hpp"
#include "Intake.hpp"
#include "Shooter.hpp"
#include "Indexer.hpp"
#include "cameraserver/CameraServer.h"
#include "RobotContainer.hpp"
#include "Auton/Auton.hpp"

#include <frc2/command/Command.h>

enum ControllerButtons {
    INDEXING = 1,
    SPOOLING = 2,
    INTAKE_OUT_1 = 5,
    INTAKE_OUT_2 = 6,
};

enum ControllerAxes {
    HUB_SHOOTING = 2,
    LAUNCHPAD_SHOOTING = 3
  };

enum ControllerPOVs {
    MANUAL_TOP = 0,
    MANUAL_REVERSE_TOP = 90,
    MANUAL_BOTTOM = 180,
    MANUAL_BOTH = 270
};

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

  void ProcessButtons();

  private:

  DriveBase *m_drive;
  Climber *m_climber;
  AHRS *m_ahrs;
  std::shared_ptr<Shooter> m_shooter;
  std::shared_ptr<Indexer> m_indexer;
  std::shared_ptr<Hood> m_hood;
  std::shared_ptr<Intake> m_intake;
  std::shared_ptr<frc::Compressor> m_compressor;
  frc::Joystick *m_joy_op;
  frc::Joystick *m_joy_drive;
  RobotContainer *m_container;
  AutonDrive *m_autondrive;

  frc2::Command* m_AutonomousCommand = nullptr;

  bool m_climb_time = false;
  bool m_climb_mode = false;

  bool  intake_button, index_button, spooling_button, 
        hub_shooting_trigger, launchpad_shooting_trigger,
        manual_reverse_top_pov, manual_top_pov, manual_bottom_pov, manual_both_pov; 

};
