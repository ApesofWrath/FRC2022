#pragma once

#include <ctre/Phoenix.h>
#include <memory>

constexpr float reverseSpeed = -1;
constexpr float intakeSpeed = 1;
constexpr float waitingSpeed = .1;

enum class IntakeState {
    Init,
    Stop,
    Waiting,
    Intake,
    Reverse
};

class Intake {
public:

    Intake();
    void stop();
    void waiting();
    void intake();
    void reverse();

    void IntakeStateMachine();

    void setState(IntakeState state) {m_LastState = m_State; m_State = state;};
    IntakeState getState();

private:
    std::shared_ptr<TalonFX> m_Motor;

    IntakeState m_LastState, m_State = IntakeState::Init;

};