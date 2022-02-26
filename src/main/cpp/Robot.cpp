// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit() {
    m_shooter = std::make_shared<Shooter>();
    m_hood = std::make_shared<Hood>();
    m_intake = std::make_shared<Intake>();
    
    m_joystick = std::make_shared<frc::Joystick>(0);
    m_compressor = std::make_shared<frc::Compressor>(3, frc::PneumaticsModuleType::CTREPCM);
    
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_compressor->EnableDigital();
}
void Robot::TeleopPeriodic() {
    if (m_joystick->GetRawButton(1)) {
        m_shooter->setState(ShooterState::SHOOT);
        m_intake->setState(IntakeState::GO);
    } else if (m_joystick->GetRawButton(2)) {
        m_shooter->setState(ShooterState::WAITING);
        m_intake->setState(IntakeState::WAITING);
    } else if (m_joystick->GetRawButton(3)) {
        m_shooter->setState(ShooterState::REVERSE);
        m_intake->setState(IntakeState::REVERSE);
    } else {
        m_shooter->setState(ShooterState::STOP);
        m_intake->setState(IntakeState::STOP);
    }

    if(m_joystick->GetRawButton(5)) {
        m_hood->setState(HoodState::UP);
    } else if(m_joystick->GetRawButton(6)) {
        m_hood->setState(HoodState::DOWN);
    }

    m_hood->HoodStateMachine();
    m_shooter->ShooterStateMachine();
}

void Robot::DisabledInit() {
    
  m_compressor->Disable();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
