#include "Intake.hpp"

Intake::Intake() {
    m_solenoid = std::make_shared<frc::DoubleSolenoid>(61, frc::PneumaticsModuleType::CTREPCM, 0, 1);
    // m_right_solenoid = std::make_shared<frc::DoubleSolenoid>(3, frc::PneumaticsModuleType::CTREPCM, 5, 9);
    // m_intake_motor = std::make_shared<rev::CANSparkMax>(9, rev::CANSparkMax::MotorType::kBrushless);
    m_intake_motor = std::make_shared<TalonFX>(20);
    m_intake_motor->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 40, 0.3));
    m_intake_motor->Config_kP(0, 0.45, 50);
    // m_intake_motor->ConfigStatorCurrentLimit()
}

void Intake::Init() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
    // m_right_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::Velocity, 0.00);
}

void Intake::Go() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    // m_right_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    m_intake_motor->Set(TalonFXControlMode::Velocity, intake_rpm);
}

void Intake::Stop() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    // m_right_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::Velocity, indexing_rpm);
}

void Intake::Indexing() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    // m_left_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::Velocity, indexing_rpm);
}

void Intake::Reverse() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::Velocity, reverse_rpm);
}

void Intake::IntakeStateMachine() {
    frc::SmartDashboard::PutNumber("intake speed", m_intake_motor->GetSelectedSensorVelocity() / 2048.0 * 600.0);
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


