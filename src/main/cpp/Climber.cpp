#include "Climber.hpp"

Climber::Climber()
{
    climber_talon1 = std::make_shared<TalonFX>(31);
    climber_talon2 = std::make_shared<TalonFX>(39);

    climber_talon1->ConfigFactoryDefault();
    climber_talon2->ConfigFactoryDefault();

    // climber_talon1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 150, 150, 10));
    // climber_talon2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 150, 150, 10));

    configStatusFrames(climber_talon1);
    configStatusFrames(climber_talon2);
    climber_talon2->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255);
    climber_talon2->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 255);

    climber_talon1->Config_kP(0, 0.02428 * .8);
    climber_talon1->Config_kI(0, 0.000003);
    climber_talon1->Config_kD(0, 0.00025795 / 100);

    climber_talon1->Config_IntegralZone(0, 20000, 50);

    climber_talon1->SetInverted(false);
    climber_talon2->SetInverted(false);

    climber_talon1->SetNeutralMode(NeutralMode::Brake);
    climber_talon2->SetNeutralMode(NeutralMode::Brake);

    climber_talon2->Follow(*climber_talon1);
}

float Climber::CalculateHeight(float n) {
    return n * TICKS_PER_ROT * m_elevator_ratio;
}

void Climber::Init()
{
    climber_talon1->Set(TalonFXControlMode::PercentOutput, 0);

    climber_talon1->SetSelectedSensorPosition(0.0);
    climber_talon2->SetSelectedSensorPosition(0.0);
    frc::SmartDashboard::PutString("Climber State", "Init");
}

void Climber::Stop()
{
    climber_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
    BrakeElevtor();
    frc::SmartDashboard::PutString("Climber State", "Stop");
}

void Climber::Up()
{
    climber_talon1->SetNeutralMode(NeutralMode::Coast);
    climber_talon2->SetNeutralMode(NeutralMode::Coast);
    climber_talon1->Set(ControlMode::PercentOutput, 0.1);
    frc::SmartDashboard::PutString("Climber State", "Up");
}

void Climber::Down() {
    if(climber_talon1->GetSelectedSensorPosition() > kElevatorThreshold) {
        climber_talon1->Set(TalonFXControlMode::PercentOutput, -0.30);
    } else {
        climber_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
        current_state = States::STOP_CLIMB;
    }
    frc::SmartDashboard::PutString("Climber State", "Down");
}

void Climber::DownSlow() {
    // if(climber_talon1->GetSelectedSensorPosition() > kElevatorThreshold) {
        climber_talon1->Set(TalonFXControlMode::PercentOutput, -0.1);
    // } else {
        // climber_talon1->Set(TalonFXControlMode::PercentOutput, 0.0);
        // current_state = States::STOP_CLIMB;
    // }
    frc::SmartDashboard::PutString("Climber State", "Down Slow");
}

void Climber::Zero() {
    climber_talon1->SetSelectedSensorPosition(0.0);
    climber_talon2->SetSelectedSensorPosition(0.0);
    climber_talon1->Set(TalonFXControlMode::PercentOutput, 0);
    frc::SmartDashboard::PutString("Climber State", "Zero");
    current_state = States::STOP_CLIMB;
}

void Climber::configStatusFrames(std::shared_ptr<TalonFX> motorController) {
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
}

int ticksee = 0;

void Climber::climberStateMachine()
{

    
    // Talon1 Smart Dashboard
     frc::SmartDashboard::PutNumber("Talon1 Voltage", climber_talon1->GetMotorOutputVoltage());
     frc::SmartDashboard::PutNumber("Talon1 Percent Out", climber_talon1->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Climb Talon1", climber_talon1->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Sensor Velocity Talon1", climber_talon1->GetSelectedSensorVelocity());


    // Talon2 Smart Dashboard
     frc::SmartDashboard::PutNumber("Talon2 Voltage", climber_talon2->GetMotorOutputVoltage());
     frc::SmartDashboard::PutNumber("Talon2 Percent Out", climber_talon2->GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Sensor Pos Climb Talon2", climber_talon2->GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Sensor Velocity Talon2", climber_talon2->GetSelectedSensorVelocity());

    frc::SmartDashboard::PutNumber("Curr State", (int)current_state);

    // frc::SmartDashboard::PutNumber("Climber State", static_cast<uint32_t>(current_state));


    switch (current_state)
    {
    
    case States::INIT:
        Init();
        current_state = States::ZERO_CLIMB;
        last_state = States::INIT;
        frc::SmartDashboard::PutString("Climber Stateisms", "Init");
        break;
    
    
    case States::STOP_CLIMB:
        Stop();
        last_state = States::STOP_CLIMB;
        frc::SmartDashboard::PutString("Climber Stateisms", "Stop");
        break;

    case States::UP_CLIMB:
        if (last_state != States::UP_CLIMB) {
            Up();
            ticksee = 0;
        }
        ticksee++;
        if (ticksee > 4) {
            climber_talon1->Set(ControlMode::PercentOutput, 0.0);
        }
        frc::SmartDashboard::PutString("Climber Stateisms", "Up");
        last_state = States::UP_CLIMB;
        break;
    case States::DOWN_CLIMB:
        if (last_state != States::DOWN_CLIMB) {
            climber_talon1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 140, 140, 5.0));
            climber_talon2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 140, 140, 5.0));
        }
        frc::SmartDashboard::PutString("Climber Stateisms", "Down");
        Down();
        last_state = States::DOWN_CLIMB;
        break;

    case States::DOWN_SLOW_CLIMB:
        if (last_state != States::DOWN_SLOW_CLIMB) {
            climber_talon1->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 140, 140, 5.0));
            climber_talon2->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 140, 140, 5.0));
        }
        DownSlow();
        frc::SmartDashboard::PutString("Climber Stateisms", "Down Slow");
        last_state = States::DOWN_SLOW_CLIMB;
        break;


    case States::ZERO_CLIMB:
        if (last_state != States::ZERO_CLIMB) {
            Zero();
        }
        last_state = States::ZERO_CLIMB;
        frc::SmartDashboard::PutString("Climber Stateisms", "Zero");
        break;
    }
}

void Climber::CoastElevator() {
    climber_talon1->SetNeutralMode(NeutralMode::Coast);
    climber_talon2->SetNeutralMode(NeutralMode::Coast);
}

void Climber::BrakeElevtor() {
    climber_talon1->SetNeutralMode(NeutralMode::Brake);
    climber_talon2->SetNeutralMode(NeutralMode::Brake);
}
