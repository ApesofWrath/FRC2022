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
    SHOOT,
    MANUALTOP,
    MANUALBOTTOM,
    MANUALBOTH,
    STOP
};

class Indexer {
public:

    Indexer();
    void Init();
    void Waiting();
    void Reverse();
    void Intake();
    void Shoot();
    void ManualTop();
    void ManualBottom();
    void ManualBoth();
    void Stop();

    void IndexerStateMachine();
    
    void setState(IndexerState state) {m_last_state = m_state; m_state = state;};
    IndexerState GetState();

    double reverseSpeed = -0.5;
    double intakeSpeed = 0.25;
    double shooterSpeed = 0.25;
    double waitingSpeed = 0.1;
 
private:
    std::shared_ptr<TalonFX> m_top_indexer;
    std::shared_ptr<TalonFX> m_bottom_indexer;
    
    frc::DigitalInput top_sensor{1};
    frc::DigitalInput bottom_sensor{2};

    IndexerState m_last_state, m_state = IndexerState::INIT;
};