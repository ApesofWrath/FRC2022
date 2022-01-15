#include <Shooter.hpp>

Shooter::Shooter() {
    
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
            m_LastState = ShooterState::Init;
            m_State = ShooterState::Stop;
        break;
        case ShooterState::Stop:
            if (m_LastState != ShooterState::Stop) {
                stop();
            }
            m_LastState = ShooterState::Stop;
        break;
        case ShooterState::Shoot:
            if (m_LastState != ShooterState::Stop) {
                shoot();
            }
            m_LastState = ShooterState::Stop;
        break;
        case ShooterState::Waiting:
            if (m_LastState != ShooterState::Waiting) {
                waiting();
            }
            m_LastState = ShooterState::Waiting;
        break;
        case ShooterState::Reverse:
            if (m_LastState != ShooterState::Reverse) {
                reverse();
            }
            m_LastState = ShooterState::Reverse;
        break;
    }
}