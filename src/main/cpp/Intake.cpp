#include "Intake.hpp"

Intake::Intake() {
    m_left_solenoid = std::make_shared<frc::DoubleSolenoid>(3, frc::PneumaticsModuleType::CTREPCM, 4, 8);
    m_right_solenoid = std::make_shared<frc::DoubleSolenoid>(3, frc::PneumaticsModuleType::CTREPCM, 5, 9);
    m_intake_spark = std::make_shared<rev::CANSparkMax>(22, rev::CANSparkMax::MotorType::kBrushless);
}
void Intake::Init() {
    m_left_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_right_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_spark->Set(0.00);
}
void Intake::Go() {
    
    m_left_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    m_right_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    m_intake_spark->Set(0.75); 
}

void Intake::Stop() {
    m_left_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_right_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_spark->Set(0.00);
}

void Intake::Waiting() {
    m_left_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
    m_right_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
    m_intake_spark->Set(0.10);
}

void Intake::Reverse() {
    m_left_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_left_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_spark->Set(-0.75);
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
        case IntakeState::WAITING:
            frc::SmartDashboard::PutString("IntakeState", "Waiting");
            if (m_last_state != IntakeState::WAITING) {
                Waiting();
            }
            m_last_state = IntakeState::WAITING;
        break;
        case IntakeState::GO:
            frc::SmartDashboard::PutString("IntakeState", "Intake");
            if (m_last_state != IntakeState::GO) {
                Intake();
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



