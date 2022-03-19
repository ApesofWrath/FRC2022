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
    REVERSE,
    INDEXING
};

class Intake {
public:

    Intake();
    void Init();
    void Stop();
    void Waiting();
    void Go();
    void Reverse();
    void Indexing();

    void IntakeStateMachine();

    void setState(IntakeState state) {m_last_state = m_state; m_state = state;};
    IntakeState getState();

    inline bool isExtended() const noexcept { return m_state == IntakeState::GO; };

    double reverseSpeed = -1.0;
    double intakeSpeed = 1.0;
    double waitingSpeed = 0.1;

private:
    std::shared_ptr<frc::DoubleSolenoid> m_solenoid;
    std::shared_ptr<frc::DoubleSolenoid> m_right_solenoid;

    std::shared_ptr<TalonFX> m_intake_motor;

    IntakeState m_last_state, m_state = IntakeState::INIT;

};