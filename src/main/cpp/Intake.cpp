#include "Intake.hpp"

constexpr int LOOPS_S = 10;

Intake::Intake() {
    m_solenoid = std::make_shared<frc::DoubleSolenoid>(61, frc::PneumaticsModuleType::CTREPCM, 0, 1);
    // m_right_solenoid = std::make_shared<frc::DoubleSolenoid>(3, frc::PneumaticsModuleType::CTREPCM, 5, 9);
    // m_intake_motor = std::make_shared<rev::CANSparkMax>(9, rev::CANSparkMax::MotorType::kBrushless);
    m_intake_motor = std::make_shared<TalonFX>(20);
    m_intake_motor->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(false, 40, 40, 0.3));
    m_intake_motor->Config_kP(0, 0.45, 50);

    configStatusFrames(m_intake_motor);
    // m_intake_motor->ConfigStatorCurrentLimit()
}

void Intake::Init() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
    // m_right_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::Velocity, indexing_rpm);
}

void Intake::Go() {
    m_intake_motor->Set(TalonFXControlMode::Velocity, intake_rpm);

    if (loopsSinceLastTransition <= 0) {
        m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        loopsSinceLastTransition--;
    }
}

void Intake::Stop() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    // m_right_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    if (loopsSinceLastTransition <= 0) {
        m_intake_motor->Set(TalonFXControlMode::Velocity, indexing_rpm);
    } else {
        loopsSinceLastTransition--;
    }
}

void Intake::Indexing() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    // m_left_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    if (loopsSinceLastTransition <= 0) {
        m_intake_motor->Set(TalonFXControlMode::Velocity, indexing_rpm);
    } else {
        loopsSinceLastTransition--;
    }
}

void Intake::Reverse() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::Velocity, reverse_rpm);
}

void Intake::configStatusFrames(std::shared_ptr<TalonFX> motorController) {
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 255);
    motorController->SetControlFramePeriod(Control_6_MotProfAddTrajPoint, 255);
}

void Intake::IntakeStateMachine() {
    // frc::SmartDashboard::PutNumber("intake speed", m_intake_motor->GetSelectedSensorVelocity() / 2048.0 * 600.0);
    switch (m_state) {
        case IntakeState::INIT:
            // frc::SmartDashboard::PutString("IntakeState", "Init");
            m_last_state = IntakeState::INIT;
            m_state = IntakeState::INIT;
            break;
        case IntakeState::STOP:
            // frc::SmartDashboard::PutString("IntakeState", "Stop");
            if (m_last_state != IntakeState::STOP) {
                loopsSinceLastTransition = LOOPS_S;
            }
            Stop();
            m_last_state = IntakeState::STOP;
            break;
        case IntakeState::INDEXING:
            // frc::SmartDashboard::PutString("IntakeState", "Waiting");
            if (m_last_state != IntakeState::INDEXING) {
                loopsSinceLastTransition = LOOPS_S;
            }
            Indexing();
            m_last_state = IntakeState::INDEXING;
            break;
        case IntakeState::GO:
            // frc::SmartDashboard::PutString("IntakeState", "Intake");
            if (m_last_state != IntakeState::GO) {
                loopsSinceLastTransition = LOOPS_S;
            }
            Go();
            m_last_state = IntakeState::GO;
            break;
        case IntakeState::REVERSE:
            // frc::SmartDashboard::PutString("IntakeState", "Reverse");
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


