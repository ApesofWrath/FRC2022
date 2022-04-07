#pragma once

#include <ctre/Phoenix.h>
#include <memory>
#include <iostream>
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
    MANUALREVERSETOP,
    MANUALBOTTOM,
    MANUALBOTH,
    SHOOTERCHECK,
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
    void ManualReverseTop();
    void ShooterCheck();
    void Stop();
    void configStatusFrames(std::shared_ptr<TalonFX> motorController);
    void IndexerStateMachine();
    
    void SetState(IndexerState state) {m_last_state = m_state; m_state = state;};
    IndexerState GetState();

    double reverseSpeed = -0.15;
    double reverse_rpm = -900.0 * 2048.0 / 600.0;
    
    double intakeSpeed = 0.15;
    double intake_rpm = 900.0 * 2048.0 / 600.0;

    double shooterSpeed = 0.15;
    double shooter_rpm = 1000.0 * 2048.0 / 600;

    double waitingSpeed = 0.0;

    double desired_ticks = 2048.0 * -0.25;
    double desired_position = 0.0;

    bool finished_top = false;

    IndexerState m_last_state, m_state = IndexerState::INIT;
 
private:
    std::shared_ptr<TalonFX> m_top_motor;
    std::shared_ptr<TalonFX> m_bottom_motor;

    std::shared_ptr<frc::DigitalInput> top_input;
    std::shared_ptr<frc::DigitalInput> bottom_input;

    std::shared_ptr<Shooter> m_shooter;
    std::shared_ptr<::Intake> m_intake;
};