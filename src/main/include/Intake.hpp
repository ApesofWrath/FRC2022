#pragma once

#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <memory>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"

enum class IntakeState {
    INIT,
    STOP,
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

    double indexingSpeed = 0.10;
    double indexing_rpm = 600.0;

    double reverseSpeed = -1.0;
    double reverse_rpm = 6000.0;

    double intakeSpeed = 0.25;
    double intake_rpm = 1500.0;
    
    double waitingSpeed = 0.10;
    double waiting_rpm = 600.0;

    std::shared_ptr<TalonFX> m_intake_motor;

private:
    std::shared_ptr<frc::DoubleSolenoid> m_solenoid;
    std::shared_ptr<frc::DoubleSolenoid> m_right_solenoid;

    IntakeState m_last_state, m_state = IntakeState::INIT;

};