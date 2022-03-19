#pragma once
#include <string>
#include <frc/smartdashboard/SendableChooser.h>

#include "Drive/DriveBase.hpp"
#include "Auton/Auton.hpp"

const std::string kAutoNameDefault = "Default";
const std::string kAutoNameCustom = "My Auto";  
const std::string kAutoName_TestPath = "Test Path";
const std::string kAutoName_CIL = "Cross Initial Line";
const std::string kAutoName_ShootPreload = "Shoot Preload";

enum Bozo {
        CROSS_INIT_LINE,
        SHOOT_PRELOAD,
        THREE_BALL,
        FIVE_BALL
};

class RobotContainer {
 public:
  RobotContainer(AutonDrive* drive) : m_drive(drive) {};

  AutonDrive* m_drive;

  void InitAutoChoices();

  frc2::Command* GetAutonomousCommand();
  frc::SendableChooser<std::string> m_chooser;
  
  std::string m_autoSelected;
 private:
  void ConfigureButtonBindings();



};