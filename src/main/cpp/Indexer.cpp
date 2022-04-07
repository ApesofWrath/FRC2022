#include "Indexer.hpp"

// Init: duh
// Waiting: set motors to slow to keep balls in index
// Reverse: reverse both motors
// Index 1st: move to top
// Index 2nd: move to bottom
// Shoot top: move top ball, then move bottom ball to top

Indexer::Indexer(const std::shared_ptr<Shooter> &shooter, const std::shared_ptr<::Intake> &intake) : m_shooter(shooter), m_intake(intake)
{
    m_bottom_motor = std::make_shared<TalonFX>(26);
    m_top_motor = std::make_shared<TalonFX>(25);

    m_bottom_motor->ConfigFactoryDefault();
    m_top_motor->ConfigFactoryDefault();

    configStatusFrames(m_bottom_motor);
    configStatusFrames(m_top_motor);

    m_bottom_motor->SetNeutralMode(NeutralMode::Brake);
    m_top_motor->SetNeutralMode(NeutralMode::Brake);

    m_top_motor->Config_kP(0, 0.086076 * 2, 50);
    m_bottom_motor->Config_kP(0, 0.086076 * 2, 50);

    bottom_input = new frc::DigitalInput(0);
    top_input = new frc::DigitalInput(1);

    // use percentoutput instead of pid
}

/**
 * @param m_bottom_motor: top motor
 * @param m_top_motor: bottom motor
 * @param bottom_input: top sensor
 * @param top_input: bottom sensor
 **/
void Indexer::Init()
{
    m_bottom_motor->SetInverted(false);
    m_top_motor->SetInverted(false);
}

void Indexer::Waiting()
{
    m_bottom_motor->Set(ControlMode::Velocity, waitingSpeed);
    m_intake->setBottomIndexRunning(false);
    m_top_motor->Set(ControlMode::Velocity, waitingSpeed);
}

void Indexer::Reverse()
{
    // m_bottom_motor->Set(ControlMode::PercentOutput, reverseSpeed);
    // m_top_motor->Set(ControlMode::PercentOutput, reverseSpeed);
    m_bottom_motor->Set(TalonFXControlMode::Velocity, reverse_rpm);
    m_top_motor->Set(TalonFXControlMode::Velocity, reverse_rpm);
}

void Indexer::Intake()
{
    // no balls :(
    if (!bottom_input->Get() && !top_input->Get())
    {
        m_bottom_motor->Set(ControlMode::Velocity, intake_rpm);
        m_intake->setBottomIndexRunning(true);
        m_top_motor->Set(ControlMode::Velocity, intake_rpm);
        // m_bottom_motor->Set(ControlMode::PercentOutput, intakeSpeed);
        // m_top_motor->Set(ControlMode::PercentOutput, intakeSpeed);
    }
    // one ball in (lower slot) :/
    else if (bottom_input->Get() && !top_input->Get())
    {
        m_bottom_motor->Set(ControlMode::Velocity, intake_rpm);
        m_intake->setBottomIndexRunning(true);
        m_top_motor->Set(ControlMode::Velocity, intake_rpm);
        finished_top = false;
        // m_bottom_motor->Set(ControlMode::PercentOutput, intakeSpeed);
        // m_top_motor->Set(ControlMode::PercentOutput, intakeSpeed);
    }
    // one ball (upper slot) :)
    else if (!bottom_input->Get() && top_input->Get())
    {
        // if(!finished_top) {
        //     desired_position = m_top_motor->GetSelectedSensorPosition() + desired_ticks;
        //     finished_top = true;
        // }
        // if(m_top_motor->GetSelectedSensorPosition() < desired_position){
        m_top_motor->Set(ControlMode::PercentOutput, 0.0);
        // } else {
        // m_top_motor->Set(ControlMode::PercentOutput, -0.1);
        // }
        m_intake->setBottomIndexRunning(true);
        m_bottom_motor->Set(ControlMode::Velocity, intake_rpm);
        // m_bottom_motor->Set(ControlMode::PercentOutput, intakeSpeed);
        // m_top_motor->Set(ControlMode::PercentOutput, 0);
    }
    // two balls :3
    else
    {
        m_bottom_motor->Set(ControlMode::PercentOutput, 0.0);
        m_top_motor->Set(ControlMode::PercentOutput, 0.0);
    }
}

void Indexer::ManualBoth()
{
    m_bottom_motor->Set(ControlMode::Velocity, intake_rpm);
    m_top_motor->Set(ControlMode::Velocity, intake_rpm);
}

void Indexer::ManualTop()
{
    m_top_motor->Set(ControlMode::Velocity, intake_rpm);
    m_bottom_motor->Set(TalonFXControlMode::PercentOutput, 0.0);
}

void Indexer::ManualBottom()
{
    m_top_motor->Set(TalonFXControlMode::PercentOutput, 0);
    m_bottom_motor->Set(ControlMode::Velocity, intake_rpm);
}

void Indexer::Shoot()
{
    if (m_shooter->readyToShoot())
    {
        // run top motor to push 1st ball into shooter
        if (top_input->Get())
        {
            m_bottom_motor->Set(ControlMode::PercentOutput, 0.0);
            m_top_motor->Set(TalonFXControlMode::Velocity, shooter_rpm);
            m_intake->setBottomIndexRunning(false);
            // m_top_motor->Set(ControlMode::PercentOutput, shooterSpeed);
            m_shooter->loopsCooldown = 80;
        }
    }
    // turn off top motor
    // then run bottom motor to push other ball into top slot
    if (bottom_input->Get() && !top_input->Get())
    {
        m_top_motor->Set(ControlMode::PercentOutput, 0.0);
        m_bottom_motor->Set(TalonFXControlMode::Velocity, shooter_rpm);
        m_intake->setBottomIndexRunning(true);
        // m_bottom_motor->Set(ControlMode::PercentOutput, shooterSpeed);
        if (!m_intake->isExtended())
        {
            m_intake->setState(IntakeState::INDEXING);
        }
    }

    if(!bottom_input->Get() && !top_input->Get()) {
        m_top_motor->Set(TalonFXControlMode::Velocity, shooter_rpm);
        m_bottom_motor->Set(TalonFXControlMode::Velocity, shooter_rpm);
        m_intake->setBottomIndexRunning(true);
        if (!m_intake->isExtended())
        {
            m_intake->setState(IntakeState::INDEXING);
        }
    }
}

void Indexer::ManualReverseTop()
{
    m_top_motor->Set(TalonFXControlMode::PercentOutput, -0.2);
    m_bottom_motor->Set(TalonFXControlMode::PercentOutput, -0.2);
}

void Indexer::ShooterCheck()
{
    if(m_top_motor->GetOutputCurrent() < 45.0) {
        m_top_motor->Set(TalonFXControlMode::Velocity, -intake_rpm / 2);
        m_bottom_motor->Set(TalonFXControlMode::PercentOutput, 0.0);
        m_shooter->setIndexerReady(false);
    } else {
        m_top_motor->Set(TalonFXControlMode::PercentOutput, 0.0);
        m_bottom_motor->Set(TalonFXControlMode::PercentOutput, 0.0);
        m_state = IndexerState::SHOOT;
        m_shooter->setIndexerReady(true); 
    }
}

void Indexer::configStatusFrames(std::shared_ptr<TalonFX> motorController)
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

void Indexer::IndexerStateMachine()
{
    frc::SmartDashboard::PutNumber("topindexcurrent", m_top_motor->GetOutputCurrent());
    if (m_last_state == IndexerState::SHOOT && m_state != IndexerState::SHOOT && m_intake->getState() == IntakeState::INDEXING)
    {
        m_intake->setState(IntakeState::STOP);
    }
    frc::SmartDashboard::PutBoolean("bot", bottom_input->Get());
    frc::SmartDashboard::PutBoolean("top", top_input->Get());

    // frc::SmartDashboard::PutNumber("ind top speed", m_bottom_motor->GetSelectedSensorVelocity() / 2048.0 * 600.0);
    // frc::SmartDashboard::PutNumber("ind bot speed", m_top_motor->GetSelectedSensorVelocity() / 2048.0 * 600.0);

    switch (m_state)
    {
    case IndexerState::INIT:
        frc::SmartDashboard::PutString("IndexState", "Init");
        m_last_state = IndexerState::INIT;
        m_state = IndexerState::INIT;
        break;
    case IndexerState::WAITING:
        frc::SmartDashboard::PutString("IndexState", "Waiting");
        if (m_last_state != IndexerState::WAITING)
        {
            Waiting();
        }
        m_last_state = IndexerState::WAITING;
        break;
    case IndexerState::REVERSE:
        frc::SmartDashboard::PutString("IndexState", "Reverse");
        if (m_last_state != IndexerState::REVERSE)
        {
            Reverse();
        }
        m_last_state = IndexerState::REVERSE;
        break;
    case IndexerState::INTAKE:
        frc::SmartDashboard::PutString("IndexState", "Intake");
        Intake();
        m_last_state = IndexerState::INTAKE;
        break;
    case IndexerState::SHOOT:
        frc::SmartDashboard::PutString("IndexState", "Shoot");
        Shoot();
        m_last_state = IndexerState::SHOOT;
        break;
    case IndexerState::MANUALREVERSETOP:
        frc::SmartDashboard::PutString("IndexState", "Manual Reverse Top");
        ManualReverseTop();
        m_last_state = IndexerState::MANUALREVERSETOP;
    case IndexerState::MANUALBOTH:
        ManualBoth();
        m_last_state = IndexerState::MANUALBOTH;
        break;
    case IndexerState::MANUALTOP:
        ManualTop();
        m_last_state = IndexerState::MANUALTOP;
        break;
    case IndexerState::MANUALBOTTOM:
        ManualBottom();
        m_last_state = IndexerState::MANUALBOTTOM;
        break;
    case IndexerState::SHOOTERCHECK:
        frc::SmartDashboard::PutString("IndexState", "Shooter Check");
        if(m_last_state != IndexerState::SHOOTERCHECK) {
            desired_position = m_top_motor->GetSelectedSensorPosition() - 2048.0 * 0.25;
        }
        ShooterCheck();
        m_last_state = IndexerState::SHOOTERCHECK;
        break;
    }
}

IndexerState Indexer::GetState()
{
    return m_state;
}