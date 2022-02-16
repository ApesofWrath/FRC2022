#include <Hood.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Robot.hpp>

// use buttons Y and B for hood adjustment (Button IDs 4 and 2\)

Hood::Hood() {
    // Use PCM channels 3 and 6 for the hood solenoid
    m_DoubleSolenoid = new frc::DoubleSolenoid(3, frc::PneumaticsModuleType::CTREPCM, 3, 6);
}

// does not move hood
void Hood::init(){
    m_DoubleSolenoid->Set(frc::DoubleSolenoid::Value::kOff);
}

// moves hood up
void Hood::up(){
    m_DoubleSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
}

// moves hood down
void Hood::down(){
    m_DoubleSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
}


void Hood::HoodStateMachine() {
    switch (m_State) {
        case HoodState::Init:
            frc::SmartDashboard::PutString("HoodState", "Init");
            m_LastState = HoodState::Init;
            m_State = HoodState::Init;
        break;
        case HoodState::Up:
            frc::SmartDashboard::PutString("HoodState", "Up");
            if (m_LastState != HoodState::Up) {
                up();
            }
            m_LastState = HoodState::Up;
        break;
        case HoodState::Down:
            frc::SmartDashboard::PutString("HoodState", "Down");
            if (m_LastState != HoodState::Down) {
                down();
            }
            m_LastState = HoodState::Down;
        break;
    }
}

HoodState Hood::getState() {
    return m_State;
}