// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit() {
    m_Shooter = std::make_shared<Shooter>();
    m_Joystick = std::make_shared<frc::Joystick>(0);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
    if (m_Joystick->GetRawButton(1)) {
        m_Shooter->setState(ShooterState::Shoot);
    } else if (m_Joystick->GetRawButton(2)) {
        m_Shooter->setState(ShooterState::Waiting);
    } else if (m_Joystick->GetRawButton(3)) {
        m_Shooter->setState(ShooterState::Reverse);
    } else {
        m_Shooter->setState(ShooterState::Stop);
    }

    m_Shooter->shooterStateMachine();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
