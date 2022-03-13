#include "Indexer.hpp"


// Init: duh
// Waiting: set motors to slow to keep balls in index
// Reverse: reverse both motors
// Index 1st: move to top
// Index 2nd: move to bottom
// Shoot top: move top ball, then move bottom ball to top


Indexer::Indexer() {
    m_motor1 = std::make_shared<TalonFX>(25);
    m_motor2 = std::make_shared<TalonFX>(26);

    m_motor1->ConfigFactoryDefault();
    m_motor2->ConfigFactoryDefault();

    m_motor1->SetNeutralMode(NeutralMode::Coast);
    m_motor2->SetNeutralMode(NeutralMode::Coast);

    //use percentoutput instead of pid
}


/**
* @param m_motor1: top motor
* @param m_motor2: bottom motor
* @param input1: top sensor
* @param input2: bottom sensor
**/
void Indexer::Init() {
    m_motor1->SetInverted(false);
    m_motor2->SetInverted(false);
}

void Indexer::Waiting() {
    m_motor1->Set(ControlMode::PercentOutput, waitingSpeed);
    m_motor2->Set(ControlMode::PercentOutput, waitingSpeed);
}

void Indexer::Reverse() {
    m_motor1->Set(ControlMode::PercentOutput, reverseSpeed);
    m_motor2->Set(ControlMode::PercentOutput, reverseSpeed);
}

void Indexer::Intake() {
    // no balls :(
    if (!input1.Get() && !input2.Get()) {
        m_motor1->Set(ControlMode::PercentOutput, intakeSpeed);
    } 
    // one ball in (lower slot) :/
    else if (!input1.Get() && input2.Get()) {
        m_motor1->Set(ControlMode::PercentOutput, intakeSpeed);
        m_motor2->Set(ControlMode::PercentOutput, intakeSpeed);
    } 
    // one ball (upper slot) :)
    else if(input1.Get() && !input2.Get()) {
        m_motor2->Set(ControlMode::PercentOutput, intakeSpeed);
    } 
    // two balls :3 ToT ^w^ //nya// x_x >:3 7w7 =3= owo7
    else {
        m_motor1->Set(ControlMode::PercentOutput, 0);
        m_motor2->Set(ControlMode::PercentOutput, 0);
    }
}

void Indexer::Shoot(){
    //run top motor to push 1st ball into shooter
    if (input1.Get() && input2.Get()) {
        m_motor1->Set(ControlMode::PercentOutput, shooterSpeed);
    }
    //turn off top motor
    m_motor1->Set(ControlMode::PercentOutput, 0);
    //then run bottom motor to push other ball into top slot
    if (!input1.Get() && input2.Get()) {
        m_motor2->Set(ControlMode::PercentOutput, shooterSpeed);   
    }
}

void Indexer::IndexerStateMachine()
{
    switch (m_state) {
    case IndexerState::INIT: 
            frc::SmartDashboard::PutString("IndexState", "Init");
            m_last_state = IndexerState::INIT;
            m_state = IntakeState::INIT;
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
    }
}

IndexerState Indexer::GetState()
{
    return m_state;
}

void Indexer::SetState(IndexerState state)
{
    m_state = state;
}