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
  std::shared_ptr<Intake> m_intake;
  std::shared_ptr<Indexer> m_indexer;
  std::shared_ptr<Shooter> m_shooter;
  std::shared_ptr<Hood> m_hood;

  void InitAutoChoices();

  frc2::Command* GetAutonomousCommand();
  frc::SendableChooser<Auto> m_chooser;
  
  Auto m_autoSelected;
 private:
  void ConfigureButtonBindings();



};