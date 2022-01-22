#include <Hood.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

// use buttons 4 and 5 for hood adjustment

Hood::Hood() {
    m_Motor = std::make_shared<TalonFX>(0);
}

void Hood::HoodStateMachine() {
    switch (m_State) {
        case HoodState::Init:
            frc::SmartDashboard::PutString("HoodState", "Init");
            m_LastState = HoodState::Init;
            m_State = HoodState::Init;
        break;
        case HoodState::LowerWall:
            frc::SmartDashboard::PutString("HoodState", "LowerWall");
            if (m_LastState != HoodState::LowerWall) {

            }
            m_LastState = HoodState::LowerWall;
        break;
        case HoodState::UpperFarWall:
            frc::SmartDashboard::PutString("HoodState", "UpperFarWall");
            if (m_LastState != HoodState::UpperFarWall) {
                
            }
            m_LastState = HoodState::UpperFarWall;
        break;
        case HoodState::UpperLaunchpad:
            frc::SmartDashboard::PutString("HoodState", "UpperLaunchpad");
            if (m_LastState != HoodState::UpperLaunchpad) {
                
            }
            m_LastState = HoodState::UpperLaunchpad;
        break;
        case HoodState::UpperWall:
            frc::SmartDashboard::PutString("HoodState", "UpperWall");
            if (m_LastState != HoodState::UpperWall) {
              
            }
            m_LastState = HoodState::UpperWall;
        break;
    }
}

HoodState Hood::getState() {
    return m_State;
}