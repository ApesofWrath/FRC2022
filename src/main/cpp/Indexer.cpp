#include "Indexer.hpp"


// Init: duh
// Waiting: set motors to slow to keep balls in index
// Reverse: reverse both motors
// Index: move 1st ball to top and 2nd ball to bottom
// Shoot top: move top ball, then move bottom ball to top


Indexer::Indexer() {
    m_top_indexer = std::make_shared<TalonFX>(25);
    m_bottom_indexer = std::make_shared<TalonFX>(26);

    m_top_indexer->ConfigFactoryDefault();
    m_bottom_indexer->ConfigFactoryDefault();

    m_top_indexer->SetNeutralMode(NeutralMode::Coast);
    m_bottom_indexer->SetNeutralMode(NeutralMode::Coast);

    //use percentoutput instead of pid
}

/**
* @param m_top_indexer: top motor
* @param m_bottom_indexer: bottom motor
* @param top_sensor: top sensor
* @param bottom_sensor: bottom sensor
**/
void Indexer::Init() {
    m_top_indexer->SetInverted(false);
    m_bottom_indexer->SetInverted(false);
}

void Indexer::Waiting() {
    m_top_indexer->Set(ControlMode::PercentOutput, waitingSpeed);
    m_bottom_indexer->Set(ControlMode::PercentOutput, waitingSpeed);
}

void Indexer::Reverse() {
    m_top_indexer->Set(ControlMode::PercentOutput, reverseSpeed);
    m_bottom_indexer->Set(ControlMode::PercentOutput, reverseSpeed);
}

void Indexer::Intake() {
    // no balls :(
    if (!top_sensor.Get() && !bottom_sensor.Get()) {
        m_top_indexer->Set(ControlMode::PercentOutput, intakeSpeed);
    } 
    // one ball in (lower slot) :/
    else if (!top_sensor.Get() && bottom_sensor.Get()) {
        m_top_indexer->Set(ControlMode::PercentOutput, intakeSpeed);
        m_bottom_indexer->Set(ControlMode::PercentOutput, intakeSpeed);
    } 
    // one ball (upper slot) :)
    else if(top_sensor.Get() && !bottom_sensor.Get()) {
        m_bottom_indexer->Set(ControlMode::PercentOutput, intakeSpeed);
    } 
    // two balls :3 ToT ^w^ //nya// x_x >:3 7w7 =3= owo7
    else {
        m_top_indexer->Set(ControlMode::PercentOutput, 0);
        m_bottom_indexer->Set(ControlMode::PercentOutput, 0);
    }
}

void Indexer::Shoot(){
    //run top motor to push 1st ball into shooter
    if (top_sensor.Get() && bottom_sensor.Get()) {
        m_top_indexer->Set(ControlMode::PercentOutput, shooterSpeed);
    }
    //turn off top motor
    m_top_indexer->Set(ControlMode::PercentOutput, 0);
    //then run bottom motor to push other ball into top slot
    if (!top_sensor.Get() && bottom_sensor.Get()) {
        m_bottom_indexer->Set(ControlMode::PercentOutput, shooterSpeed);   
    }
}

void Indexer::ManualTop() {
    m_top_indexer->Set(TalonFXControlMode::PercentOutput, 0.3);
    m_bottom_indexer->Set(TalonFXControlMode::PercentOutput, 0.0);
}

void Indexer::ManualBottom() {
    m_top_indexer->Set(TalonFXControlMode::PercentOutput, 0.0);
    m_bottom_indexer->Set(TalonFXControlMode::PercentOutput, 0.3);
}

void Indexer::ManualBoth() {
    m_top_indexer->Set(TalonFXControlMode::PercentOutput, 0.3);
    m_bottom_indexer->Set(TalonFXControlMode::PercentOutput, 0.3);
}

void Indexer::IndexerStateMachine()
{
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
        if (m_last_state != IndexerState::INTAKE) {
            Intake();
        }
        m_last_state = IndexerState::INTAKE;
        break;
    case IndexerState::SHOOT:
        frc::SmartDashboard::PutString("IndexState", "Shoot");
        if (m_last_state != IndexerState::SHOOT) {
            Shoot();
        }
        m_last_state = IndexerState::SHOOT;
        break;
    case IndexerState::MANUALTOP:
        frc::SmartDashboard::PutString("IndexState", "Manual Top");
        ManualTop();
        break;
    case IndexerState::MANUALBOTTOM:
        frc::SmartDashboard::PutString("IndexState", "ManualBottom");
        ManualBottom();
        break;
    case IndexerState::MANUALBOTH:
        frc::SmartDashboard::PutString("IndexState", "ManualBoth");
        ManualBoth();
        break;
    }
}

IndexerState Indexer::GetState()
{
    return m_state;
}