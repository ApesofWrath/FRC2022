// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit()
{
  m_Joystick = std::make_unique<frc::Joystick>(0);
  m_climber = std::make_shared<Climber>();
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
  if (m_Joystick->GetRawButton(6))
  {
    m_climber->current_state = States::UP_CLIMB;
  }
  else if (m_Joystick->GetRawButton(8))
  {
    m_climber->current_state = States::ZERO_CLIMB;
  }
  else if (m_Joystick->GetRawButton(7))
  {
    m_climber->current_state = States::STOP_CLIMB;
  }
  else if (m_Joystick->GetRawButton(5))
  {
    m_climber->current_state = States::DOWN_CLIMB;
  }
  else if (m_Joystick->GetRawButton(2))
  {
    m_climber->current_state = States::ARM_FORWARD;
  }
  else if (m_Joystick->GetRawButton(3))
  {
    m_climber->current_state = States::ARM_REVERSE;
  }

  m_climber->climberStateMachine();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
