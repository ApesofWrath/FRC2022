#include "Hood.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

Hood::Hood() {
    m_DoubleSolenoid = std::make_shared<frc::DoubleSolenoid>(61, frc::PneumaticsModuleType::CTREPCM, 2, 3);
}

// does not move hood
void Hood::Init() {
    m_DoubleSolenoid->Set(frc::DoubleSolenoid::Value::kOff);
}

// moves hood up
void Hood::Up() {
    m_DoubleSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

// moves hood down
void Hood::Down() {
    m_DoubleSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
}


void Hood::HoodStateMachine() {
    switch (m_state) {
        case HoodState::INIT:
            // frc::SmartDashboard::PutString("HoodState", "Init");
            m_last_state = HoodState::INIT;
            m_state = HoodState::INIT;
        break;
        case HoodState::UP:
            // frc::SmartDashboard::PutString("HoodState", "Up");
            if (m_last_state != HoodState::UP) {
                Up();
            }
            m_last_state = HoodState::UP;
        break;
        case HoodState::DOWN:
            // frc::SmartDashboard::PutString("HoodState", "Down");
            if (m_last_state != HoodState::DOWN) {
                Down();
            }
            m_last_state = HoodState::DOWN;
        break;
    }
}

HoodState Hood::getState() {
    return m_state;
}