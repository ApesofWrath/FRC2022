#pragma once
#include <string>
#include <frc/smartdashboard/SendableChooser.h>

#include "Drive/DriveBase.hpp"
#include "Auton/Auton.hpp"

const std::string kAutoNameDefault = "Default";
const std::string kAutoNameCustom = "My Auto";
const std::string kAutoName_TestPath = "TestPath";
const std::string kAutoName_UnnamedPath = "UnnamedPath";
const std::string kAutoName_Unnamed = "Unnamed";

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