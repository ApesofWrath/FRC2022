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
        if(cooldown < 0) {
        } else {
            cooldown--;
        }
    } else {
        loopsSinceLastTransition--;
    }
}

void Intake::Stop() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    // m_right_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    if (loopsSinceLastTransition <= 0 && bottom_index_running) {
        m_intake_motor->Set(TalonFXControlMode::Velocity, indexing_rpm);
    } else if (loopsSinceLastTransition <= 0 && !bottom_index_running) {
        m_intake_motor->Set(TalonFXControlMode::Velocity, indexing_rpm);
        cooldown = 200;
    } else {
        loopsSinceLastTransition--;
    }
}

void Intake::Indexing() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    // m_left_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    if (loopsSinceLastTransition <= 0 && bottom_index_running) {
        m_intake_motor->Set(TalonFXControlMode::Velocity, indexing_rpm);
        cooldown = 20;
    } else {
        loopsSinceLastTransition--;
    }
}

void Intake::Reverse() {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    m_intake_motor->Set(TalonFXControlMode::Velocity, reverse_rpm);
}

void Intake::OnlyOut() {
    m_solenoid->Set(frc::DoubleSolenoid::kForward);
    m_intake_motor->Set(TalonFXControlMode::PercentOutput, 0.0);
}

void Intake::configStatusFrames(std::shared_ptr<TalonFX> motorController)
{
    std::cout << "TalonFX Motor: " << motorController->GetDeviceID() << "\n";
    std::cout << "Status Frame 1: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 1000) << "\n";
    std::cout << "Status Frame 2: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 1000) << "\n";
    std::cout << "Status Frame 3: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 1000) << "\n";
    std::cout << "Status Frame 4: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 1000) << "\n";
    std::cout << "Status Frame 6: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 1000) << "\n";
    std::cout << "Status Frame 7: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 1000) << "\n";
    std::cout << "Status Frame 8: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 1000) << "\n";
    std::cout << "Status Frame 9: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 1000) << "\n";
    std::cout << "Status Frame 10: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 1000) << "\n";
    std::cout << "Status Frame 11: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 1000) << "\n";
    std::cout << "Status Frame 12: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 1000) << "\n";
    std::cout << "Status Frame 13: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 1000) << "\n";
    std::cout << "Status Frame 14: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 1000) << "\n";
    std::cout << "Status Frame 15: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 1000) << "\n";
    std::cout << "Status Frame 17: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 1000) << std::endl;
    std::cout << "Status Frame Brushless Current: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current) << "\n";
    std::cout << "Status Frame 21: " << motorController->GetStatusFramePeriod(StatusFrameEnhanced::Status_21_FeedbackIntegrated) << std::endl;

    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 255);
    motorController->SetStatusFramePeriod(StatusFrameEnhanced::Status_21_FeedbackIntegrated, 255);
    motorController->SetControlFramePeriod(Control_6_MotProfAddTrajPoint, 255);
}

void Intake::IntakeStateMachine() {
    frc::SmartDashboard::PutNumber("intake cooldown", cooldown);
    frc::SmartDashboard::PutBoolean("index bot wheel", bottom_index_running);
    // frc::SmartDashboard::PutNumber("intake speed", m_intake_motor->GetSelectedSensorVelocity() / 2048.0 * 600.0);
    switch (m_state) {
        case IntakeState::INIT:
            frc::SmartDashboard::PutString("IntakeState", "Init");
            m_last_state = IntakeState::INIT;
            m_state = IntakeState::INIT;
            break;
        case IntakeState::STOP:
            frc::SmartDashboard::PutString("IntakeState", "Stop");
            if (m_last_state != IntakeState::STOP) {
                loopsSinceLastTransition = LOOPS_S;
            }
            Stop();
            m_last_state = IntakeState::STOP;
            break;
        case IntakeState::INDEXING:
            frc::SmartDashboard::PutString("IntakeState", "Waiting");
            if (m_last_state != IntakeState::INDEXING) {
                loopsSinceLastTransition = LOOPS_S;
            }
            Indexing();
            m_last_state = IntakeState::INDEXING;
            break;
        case IntakeState::GO:
            frc::SmartDashboard::PutString("IntakeState", "Intake");
            if (m_last_state != IntakeState::GO) {
                loopsSinceLastTransition = LOOPS_S;
            }
            Go();
            m_last_state = IntakeState::GO;
            break;
        case IntakeState::REVERSE:
            frc::SmartDashboard::PutString("IntakeState", "Reverse");
            if (m_last_state != IntakeState::REVERSE) {
                Reverse();
            }
            m_last_state = IntakeState::REVERSE;
            break;
        case IntakeState::ONLY_OUT:
            frc::SmartDashboard::PutString("IntakeState", "Only out");
            OnlyOut();
            break;
        
    }
}

IntakeState Intake::getState () {
    return m_state;
}


