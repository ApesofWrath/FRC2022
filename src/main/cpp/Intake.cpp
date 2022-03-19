#include "Intake.hpp"

Intake::Intake() {
    m_solenoid = std::make_shared<frc::DoubleSolenoid>(61, frc::PneumaticsModuleType::CTREPCM, 0, 1);
    // m_right_solenoid = std::make_shared<frc::DoubleSolenoid>(3, frc::PneumaticsModuleType::CTREPCM, 5, 9);
    // m_intake_motor = std::make_shared<rev::CANSparkMax>(9, rev::CANSparkMax::MotorType::kBrushless);
    m_intake_motor = std::make_shared<TalonFX>(20);
}

void Intake::Init() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
    // m_right_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::PercentOutput, 0.00);
}

void Intake::Go() {
    
    m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    // m_right_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    m_intake_motor->Set(TalonFXControlMode::PercentOutput, 0.25); 
}

void Intake::Stop() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    // m_right_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::PercentOutput, 0.00);
}

void Intake::Indexing() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    // m_left_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::PercentOutput, 0.10);
}

void Intake::Reverse() {
    
}

void Intake::IntakeStateMachine() {
    switch (m_state) {
        case IntakeState::INIT:
            frc::SmartDashboard::PutString("IntakeState", "Init");
            m_last_state = IntakeState::INIT;
            m_state = IntakeState::INIT;
            break;
        case IntakeState::STOP:
            frc::SmartDashboard::PutString("IntakeState", "Stop");
            if (m_last_state != IntakeState::STOP) {
                Stop();
            }
            m_last_state = IntakeState::STOP;
            break;
        case IntakeState::INDEXING:
            frc::SmartDashboard::PutString("IntakeState", "Waiting");
            if (m_last_state != IntakeState::INDEXING) {
                Indexing();
            }
            m_last_state = IntakeState::INDEXING;
            break;
        case IntakeState::GO:
            frc::SmartDashboard::PutString("IntakeState", "Intake");
            if (m_last_state != IntakeState::GO) {
                Go();
            }
            m_last_state = IntakeState::GO;
            break;
        case IntakeState::REVERSE:
            frc::SmartDashboard::PutString("IntakeState", "Reverse");
            if (m_last_state != IntakeState::REVERSE) {
                Reverse();
            }
            m_last_state = IntakeState::REVERSE;
            break;
    }
}

IntakeState Intake::getState () {
    return m_state;
}


