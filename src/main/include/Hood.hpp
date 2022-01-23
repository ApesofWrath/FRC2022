#pragma once

#include <ctre/Phoenix.h>
#include <memory>
#include <unordered_map>

constexpr float hoodSpeed = .1;
constexpr int hoodMotor = 1;

enum class HoodState {
    Init,
    UpperWall,
    UpperFarWall,
    UpperLaunchpad,
    LowerWall
};


class Hood {
public:

    Hood();
    void init();
    void upperWall();
    void upperFarWall();
    void upperLaunchPad();
    void lowerWall();
    

    inline float getTargetPosition() { return m_TargetPosition; };
    void setTargetPosition(float angle);

    void HoodStateMachine();

    inline void setState(HoodState state) {m_LastState = m_State; m_State = state;}
    HoodState getState();

private:
    std::unordered_map<HoodState,float> m_PositionMap;
    std::shared_ptr<TalonFX> m_Motor;

    float m_TargetPosition;

    HoodState m_LastState, m_State = HoodState::Init;

    const float kP = 0;
    const float kI = 0;
    const float kD = 0;
    const float kF = 0;

};

