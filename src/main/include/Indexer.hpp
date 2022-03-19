#pragma once

#include <ctre/Phoenix.h>
#include <memory>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include "Shooter.hpp"
#include "Intake.hpp"


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

    Indexer(const std::shared_ptr<Shooter>& shooter, const std::shared_ptr<::Intake>& intake);
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
    
    void SetState(IndexerState state) {m_last_state = m_state; m_state = state;};
    IndexerState GetState();

    double reverseSpeed = -0.5;
    double intakeSpeed = 900.0 * 2048.0 / 600;
    double shooterSpeed = 900.0 * 2048.0 / 600;
    double waitingSpeed = 450.0 * 2048.0 / 600;
 
private:
    std::shared_ptr<TalonFX> m_top_motor;
    std::shared_ptr<TalonFX> m_bottom_motor;
    
    frc::DigitalInput *top_input;
    frc::DigitalInput *bottom_input;

    IndexerState m_last_state, m_state = IndexerState::INIT;
    std::shared_ptr<Shooter> m_shooter;
    std::shared_ptr<::Intake> m_intake;
};