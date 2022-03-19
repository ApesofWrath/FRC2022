// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

void Robot::RobotInit() {
    m_shooter = std::make_shared<Shooter>();
    m_hood = std::make_shared<Hood>();
    m_intake = std::make_shared<Intake>();
    m_indexer = std::make_shared<Indexer>(m_shooter, m_intake);
    m_compressor = std::make_shared<frc::Compressor>(61, frc::PneumaticsModuleType::CTREPCM);
    m_joy_op = new frc::Joystick(0);
    // m_drive = new DriveBase(m_joy_op);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    m_compressor->EnableDigital();
}
void Robot::TeleopPeriodic() {
    if (m_joy_op->GetRawButton(1)) {
        m_indexer->SetState(IndexerState::SHOOT);
        m_shooter->setState(ShooterState::SHOOT);
        // m_intake->setState(IntakeState::GO);
    } else if (m_joy_op->GetRawButton(2)) {
        m_shooter->setState(ShooterState::WAITING);
        // m_intake->setState(IntakeState::WAITING);
    } else if (m_joy_op->GetRawButton(3)) {
        m_shooter->setState(ShooterState::REVERSE);
        // m_intake->setState(IntakeState::REVERSE);
    } else if(m_joy_op->GetRawButton(9)) {
        m_hood->setState(HoodState::DOWN);
    } else if (m_joy_op->GetRawButton(10)) {
        m_hood->setState(HoodState::UP);
    } else if(m_joy_op->GetRawButton(5)) {
        m_indexer->SetState(IndexerState::INTAKE);
        m_intake->setState(IntakeState::GO);
    } else if(m_joy_op->GetRawButton(6)) {
        m_intake->setState(IntakeState::STOP);
    } else {
        if (m_intake->getState() != IntakeState::INDEXING)
            m_intake->setState(IntakeState::WAITING);
        m_shooter->setState(ShooterState::STOP);
        m_indexer->SetState(IndexerState::WAITING);
    }

    if(m_joy_op->GetRawButton(7)) {
        m_compressor->EnableDigital();
    } else if(m_joy_op->GetRawButton(8)) {
        m_compressor->Disable();
    }

    frc::SmartDashboard::PutNumber("Comp Press:", static_cast<float>(m_compressor->GetPressure()));
    frc::SmartDashboard::PutBoolean("swithcch", m_compressor->GetPressureSwitchValue());

    // if(m_joy_op->GetRawButton(5)) {
    //     m_hood->setState(HoodState::UP);
    // } else if(m_joy_op->GetRawButton(6)) {
    //     m_hood->setState(HoodState::DOWN);
    // }
    m_indexer->IndexerStateMachine();
    m_hood->HoodStateMachine();
    m_shooter->ShooterStateMachine();
    m_intake->IntakeStateMachine();
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
