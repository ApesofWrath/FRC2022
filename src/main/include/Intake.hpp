#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <memory>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"

enum class IntakeState {
    INIT,
    STOP,
    WAITING,
    GO,
    REVERSE
};

class Intake {
public:

    Intake();
    void Init();
    void Stop();
    void Waiting();
    void Go();
    void Reverse();

    void IntakeStateMachine();

    void setState(IntakeState state) {m_last_state = m_state; m_state = state;};
    IntakeState getState();

    double reverseSpeed = -0.3;
    double intakeSpeed = 0.3;
    double waitingSpeed = 0.0;

private:
    std::shared_ptr<TalonFX> m_intake_motor;
    std::shared_ptr<frc::DoubleSolenoid> m_solenoid;
    std::shared_ptr<frc::DoubleSolenoid> m_right_solenoid;


    IntakeState m_last_state, m_state = IntakeState::INIT;

};