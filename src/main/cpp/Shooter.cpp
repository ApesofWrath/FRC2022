#include <Shooter.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter() {
    m_Motor = std::make_shared<TalonFX>(0);
}

void Shooter::shoot() {
    m_Motor->Set(ControlMode::PercentOutput, shootSpeed);
}

void Shooter::intake() {

}

void Shooter::stop() {
    m_Motor->Set(ControlMode::PercentOutput, 0.f);

}

void Shooter::waiting() {
    m_Motor->Set(ControlMode::PercentOutput, waitingSpeed);
}

void Shooter::reverse() {
    m_Motor->Set(ControlMode::PercentOutput, reverseSpeed);
}

void Shooter::shooterStateMachine() {
    switch(m_State) {
        case ShooterState::Init:
            frc::SmartDashboard::PutString("ShooterState", "Init");
            m_LastState = ShooterState::Init;
            m_State = ShooterState::Stop;
        break;
        case ShooterState::Stop:
            frc::SmartDashboard::PutString("ShooterState", "Stop");
            if (m_LastState != ShooterState::Stop) {
                stop();
            }
            m_LastState = ShooterState::Stop;
        break;
        case ShooterState::Shoot:
            frc::SmartDashboard::PutString("ShooterState", "Shoot");
            if (m_LastState != ShooterState::Shoot) {
                shoot();
            }
            m_LastState = ShooterState::Shoot;
        break;
        case ShooterState::Waiting:
            frc::SmartDashboard::PutString("ShooterState", "Waiting");
            if (m_LastState != ShooterState::Waiting) {
                waiting();
            }
            m_LastState = ShooterState::Waiting;
        break;
        case ShooterState::Reverse:
            frc::SmartDashboard::PutString("ShooterState", "Reverse");
            if (m_LastState != ShooterState::Reverse) {
                reverse();
            }
            m_LastState = ShooterState::Reverse;
        break;
    }
}