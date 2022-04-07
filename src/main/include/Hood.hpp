#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <memory>
#include <unordered_map>

enum class HoodState {
    INIT,
    UP,
    DOWN
};


class Hood {
public:

    Hood();
    void Init();
    void Up();
    void Down();
    

    inline float getTargetPosition() { return m_TargetPosition; };
    void setTargetPosition(float angle);

    void HoodStateMachine();

    inline void setState(HoodState state) {m_last_state = m_state; m_state = state;}
    HoodState getState();

private:
    std::unordered_map<HoodState,float> m_PositionMap;
    std::shared_ptr<frc::DoubleSolenoid> m_DoubleSolenoid;

    float m_TargetPosition;

    HoodState m_last_state, m_state = HoodState::INIT;
    
};

