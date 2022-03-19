#pragma once

#include <ctre/Phoenix.h>
#include <memory>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>


enum class IndexerState {
    INIT,
    WAITING,
    REVERSE,
    INTAKE,
    SHOOT
};

class Indexer {
public:

    Indexer();
    void Init();
    void Waiting();
    void Reverse();
    void Intake();
    void Shoot();

    void IndexerStateMachine();
    
    void SetState(IndexerState state) {m_last_state = m_state; m_state = state;};
    IndexerState GetState();

    double reverseSpeed = -0.5;
    double intakeSpeed = 0.15;
    double shooterSpeed = 0.25;
    double waitingSpeed = 0.0;
 
private:
    std::shared_ptr<TalonFX> m_bottom_motor;
    std::shared_ptr<TalonFX> m_top_motor;
    
    frc::DigitalInput *bottom_input;
    frc::DigitalInput *top_input;

    IndexerState m_last_state, m_state = IndexerState::INIT;
};