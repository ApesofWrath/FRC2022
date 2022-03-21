#pragma once
#include <string>
#include <frc/smartdashboard/SendableChooser.h>

//do we need these?
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/Compressor.h>

#include "Drive/DriveBase.hpp"
#include "Climber.hpp"
#include "Hood.hpp"
#include "Intake.hpp"
#include "Shooter.hpp"
#include "Indexer.hpp"
#include "RobotContainer.hpp"
#include "Auton/Auton.hpp"
#include "cameraserver/CameraServer.h"
#include "Auton/Auton.hpp"

const std::string kAutoNameDefault = "Default";
const std::string kAutoNameCustom = "My Auto";  
const std::string kAutoName_TestPath = "Test Path";
const std::string kAutoName_CIL = "Cross Initial Line";
const std::string kAutoName_ShootPreload = "Shoot Preload";

enum Auto {
        SPIN,
        CROSS_INIT_LINE,
        SHOOT_PRELOAD,
        THREE_BALL,
        FIVE_BALL
};

class RobotContainer {
 public:
  RobotContainer(AutonDrive* drive) : m_drive(drive) {};

  AutonDrive* m_drive;
  Intake m_intake;
  Indexer m_indexer;
  Shooter m_shooter;
  Hood m_hood;

  void InitAutoChoices();

  frc2::Command* GetAutonomousCommand();
  frc::SendableChooser<Auto> m_chooser;
  
  Auto m_autoSelected;
 private:
  void ConfigureButtonBindings();



};