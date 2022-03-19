#include "Indexer.hpp"


// Init: duh
// Waiting: set motors to slow to keep balls in index
// Reverse: reverse both motors
// Index 1st: move to top
// Index 2nd: move to bottom
// Shoot top: move top ball, then move bottom ball to top


Indexer::Indexer() {
    m_bottom_motor = std::make_shared<TalonFX>(26);
    m_top_motor = std::make_shared<TalonFX>(25);

    m_bottom_motor->ConfigFactoryDefault();
    m_top_motor->ConfigFactoryDefault();

    m_bottom_motor->SetNeutralMode(NeutralMode::Brake);
    m_top_motor->SetNeutralMode(NeutralMode::Brake);

    bottom_input = new frc::DigitalInput(0);
    top_input = new frc::DigitalInput(1);

    //use percentoutput instead of pid
}


/**
* @param m_bottom_motor: top motor
* @param m_top_motor: bottom motor
* @param bottom_input: top sensor
* @param top_input: bottom sensor
**/
void Indexer::Init() {
    m_bottom_motor->SetInverted(false);
    m_top_motor->SetInverted(false);
}

void Indexer::Waiting() {
    m_bottom_motor->Set(ControlMode::PercentOutput, waitingSpeed);
    m_top_motor->Set(ControlMode::PercentOutput, waitingSpeed);
}

void Indexer::Reverse() {
    m_bottom_motor->Set(ControlMode::PercentOutput, reverseSpeed);
    m_top_motor->Set(ControlMode::PercentOutput, reverseSpeed);
}

void Indexer::Intake() {
    // no balls :(
    if (!bottom_input->Get() && !top_input->Get()) {
        m_bottom_motor->Set(ControlMode::PercentOutput, intakeSpeed);
        m_top_motor->Set(ControlMode::PercentOutput, intakeSpeed);
    } 
    // one ball in (lower slot) :/
    else if (bottom_input->Get() && !top_input->Get()) {
        m_bottom_motor->Set(ControlMode::PercentOutput, intakeSpeed);
        m_top_motor->Set(ControlMode::PercentOutput, intakeSpeed);
    } 
    // one ball (upper slot) :)
    else if(!bottom_input->Get() && top_input->Get()) {
        m_bottom_motor->Set(ControlMode::PercentOutput, intakeSpeed);
        m_top_motor->Set(ControlMode::PercentOutput, 0);
    } 
    // two balls :3 
    else {
        m_bottom_motor->Set(ControlMode::PercentOutput, 0);
        m_top_motor->Set(ControlMode::PercentOutput, 0);
    }
}

void Indexer::Shoot(){
    //run top motor to push 1st ball into shooter
    if (top_input->Get()) {
        m_top_motor->Set(ControlMode::PercentOutput, shooterSpeed);
    }
    //turn off top motor
    m_bottom_motor->Set(ControlMode::PercentOutput, 0);
    //then run bottom motor to push other ball into top slot
    if (bottom_input->Get() && !top_input->Get()) {
        m_bottom_motor->Set(ControlMode::PercentOutput, shooterSpeed);   
    } else {

    }
}

void Indexer::IndexerStateMachine()
{
    frc::SmartDashboard::PutBoolean("bot", bottom_input->Get());
    frc::SmartDashboard::PutBoolean("top", top_input->Get());
    switch (m_state) {
    case IndexerState::INIT: 
            frc::SmartDashboard::PutString("IndexState", "Init");
            m_last_state = IndexerState::INIT;
            m_state = IndexerState::INIT;
        break;
    case IndexerState::WAITING:
            frc::SmartDashboard::PutString("IndexState", "Waiting");
            if (m_last_state != IndexerState::WAITING) {
                Waiting();
            }
            m_last_state = IndexerState::WAITING;
            break;
    case IndexerState::REVERSE:
            frc::SmartDashboard::PutString("IndexState", "Reverse");
            if (m_last_state != IndexerState::REVERSE) {
                Reverse();
            }
            m_last_state = IndexerState::REVERSE;
            break;
    case IndexerState::INTAKE:
            frc::SmartDashboard::PutString("IndexState", "One Ball");
            // if (m_last_state != IndexerState::INTAKE) {
                Intake();
            // }
            m_last_state = IndexerState::INTAKE;
            break;
    case IndexerState::SHOOT:
            frc::SmartDashboard::PutString("IndexState", "Shoot");
            if (m_last_state != IndexerState::SHOOT) {
                Shoot();
            }
            m_last_state = IndexerState::SHOOT;
            break;
    }
}

IndexerState Indexer::GetState()
{
    return m_state;
}
