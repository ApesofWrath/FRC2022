#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <memory>
#include <unordered_map>

enum class HoodState {
    Init,
    Up,
    Down
};


class Hood {
public:

    Hood();
    void init();
    void up();
    void down();
    

    inline float getTargetPosition() { return m_TargetPosition; };
    void setTargetPosition(float angle);

    void HoodStateMachine();

    inline void setState(HoodState state) {m_LastState = m_State; m_State = state;}
    HoodState getState();

private:
    std::unordered_map<HoodState,float> m_PositionMap;
    frc::DoubleSolenoid *m_DoubleSolenoid;

    float m_TargetPosition;

    HoodState m_LastState, m_State = HoodState::Init;

    const float kP = 0;
    const float kI = 0;
    const float kD = 0;
    const float kF = 0;

};

