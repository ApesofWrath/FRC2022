#include <Shooter.hpp>

Shooter::Shooter() {
    
}

void Shooter::shooterStateMachine() {
    switch(m_State) {
        case ShooterState::Init:
        break;
        case ShooterState::Stop:
            stop();
        break;
        case ShooterState::Shoot:
            shoot();
        break;
        case ShooterState::Reverse:
            reverse();
        break;
    }
}