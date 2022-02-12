#pragma once

#include <ctre/Phoenix.h>
#include <memory>
#include <frc/smartdashboard/SmartDashboard.h>

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

    double reverseSpeed = -1.0;
    double intakeSpeed = 1.0;
    double waitingSpeed = 0.1;

private:
    std::shared_ptr<TalonFX> m_Motor;

    IntakeState m_LastState, m_State = IntakeState::Init;

};