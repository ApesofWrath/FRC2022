#include <Intake.hpp>
#include <Shooter.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {
    m_Motor = std::make_shared<TalonFX>(0);
}

void Shooter::intake() {
    m_Motor->Set(ControlMode::PercentOutput, intakeSpeed);
}

void Intake::stop() {
    m_Motor->Set(ControlMode::PercentOutput, 0);
}

void Intake::waiting() {
    m_Motor->Set(ControlMode::PercentOutput, waitingSpeed);
}

void Intake::reverse() {
    m_Motor->Set(ControlMode::PercentOutput, reverseSpeed);
}


void Intake::IntakeStateMachine() {
    switch (m_State) {
        case IntakeState::Init:
            frc::SmartDashboard::PutString("IntakeState", "Init");
            m_LastState = IntakeState::Init;
            m_State = IntakeState::Init;
        break;
        case IntakeState::Stop:
            frc::SmartDashboard::PutString("IntakeState", "Stop");
            if (m_LastState != IntakeState::Stop) {
                stop();
            }
            m_LastState = IntakeState::Stop;
        break;
        case IntakeState::Waiting:
            frc::SmartDashboard::PutString("IntakeState", "Waiting");
            if (m_LastState != IntakeState::Waiting) {
                waiting();
            }
            m_LastState = IntakeState::Waiting;
        break;
        case IntakeState::Intake:
            frc::SmartDashboard::PutString("IntakeState", "Intake");
            if (m_LastState != IntakeState::Intake) {
                intake();
            }
            m_LastState = IntakeState::Intake;
        break;
        case IntakeState::Reverse:
            frc::SmartDashboard::PutString("IntakeState", "Reverse");
            if (m_LastState != IntakeState::Reverse) {
                reverse();
            }
            m_LastState = IntakeState::Reverse;
        break;
    }
}

IntakeState Intake::getState () {
    return m_State;
}



