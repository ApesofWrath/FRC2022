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
    double intakeSpeed = 0.25;
    double shooterSpeed = 0.25;
    double waitingSpeed = 0.1;
 
private:
    std::shared_ptr<TalonFX> m_motor1;
    std::shared_ptr<TalonFX> m_motor2;
    
    frc::DigitalInput input1{1};
    frc::DigitalInput input2{2};

    IndexerState m_last_state, m_state = IndexerState::INIT;
}