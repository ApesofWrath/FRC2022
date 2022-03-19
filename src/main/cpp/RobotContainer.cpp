#include "RobotContainer.hpp"
#include <units/length.h>
#include <units/voltage.h>
#include <units/time.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <iostream>
#include <vector>

const float K_TRACK_WIDTH = 0.68533;//0.688975
//constexpr auto K_MAX_ACCEL = 10.77;
constexpr auto K_S = units::volt_t(0.53229);
constexpr auto K_V = 2.8908 * 1_V * 1_s / 1_m;
constexpr auto K_A = 0.11875 * 1_V * 1_s * 1_s / 1_m;
constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
constexpr auto kRamseteZeta = 0.7 / 1_rad;

constexpr auto kP_ = 0.00019388;

constexpr auto kMaxForwardVelocity = 3.870452_m / 1_s;

const frc::DifferentialDriveKinematics K_DRIVE_KINEMATICS{
    units::meter_t(K_TRACK_WIDTH)};

frc2::Command* RobotContainer::GetAutonomousCommand() {

    std::cout << "start\n";
    frc::DifferentialDriveVoltageConstraint *autoVoltageConstraint = new frc::DifferentialDriveVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meter>(K_S, K_V, K_A),
        K_DRIVE_KINEMATICS, units::volt_t(10));

    std::cout << "step 2\n";
    // Set up config for trajectory; sets max velocity/acceleraton in very nice sig figs
    frc::TrajectoryConfig *config = new frc::TrajectoryConfig(kMaxForwardVelocity,//2
                                units::meters_per_second_squared_t(2));//5
    // Add kinematics to ensure max speed is actually obeyed
    std::cout << "step 3\n";
    config->SetKinematics(K_DRIVE_KINEMATICS);
    // Apply the voltage constraint
    std::cout << "step 4\n";
    config->AddConstraint(*autoVoltageConstraint);

    frc::Pose2d start, end;
    std::vector<frc::Translation2d> points;

    frc::Trajectory trajectory, trajectory1, trajectory2, trajectoryTurn;
    frc2::RamseteCommand *ramseteCommand, *ramseteCommand1, *ramseteCommand2, *ramseteCommandTurn;



    switch (m_autoSelected){
        case CROSS_INIT_LINE:

            start = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
            end = frc::Pose2d(2_m, 0_m, frc::Rotation2d(0_deg));
            points = {};
            trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                start,
                points,
                end,
                *config);
            ramseteCommand = new frc2::RamseteCommand(
                trajectory, [this]() { return m_drive->getPose(); },
                frc::RamseteController(kRamseteB, kRamseteZeta),
                frc::SimpleMotorFeedforward<units::meters>(
                    K_S, K_V, K_A),
                K_DRIVE_KINEMATICS,
                [this] { return m_drive->getWheelSpeeds(); },
                frc2::PIDController(kP_, 0, 0),
                frc2::PIDController(kP_, 0, 0),
                [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
                {m_drive});

            m_drive->resetOdometry(start);

            return new frc2::SequentialCommandGroup(
                std::move(*ramseteCommand),
                frc2::InstantCommand([this] { 
                m_drive->tankDriveVolts(0_V, 0_V);
                std::cout << "Stop1\n"; 
                }, {})
            );

            default:
            std::cout << "df\n";
            return new frc2::InstantCommand([this] { frc::SmartDashboard::PutString("status","Why aren't you running auton??");});

        break;
    }













    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~











    std::vector<frc::Translation2d> pointSpin = {
    };

    std::vector<frc::Translation2d> points = {
    };

    std::vector<frc::Translation2d> points2 = {
        
    };
    
    std::vector<frc::Translation2d> points3 = {
        frc::Translation2d(-45.074_in, 0_in),
        frc::Translation2d(1.567_in, 101.473_in),
    };


    config->SetReversed(false);

    //shoot preload    
    auto Trajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
        points,
        frc::Pose2d(41_in, -4.140_in, frc::Rotation2d(-22.41_deg)),
        *config
    );

    config->SetReversed(true);
    //backtocenter
    auto Trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(41_in, -4.140_in, frc::Rotation2d(-22.41_deg)),
        points2,
        frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
        *config
    );

    config->SetReversed(false);
    auto Trajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(
        // get balls
        frc::Pose2d(0_in, -.1_in, frc::Rotation2d(180_deg)),
        points3,
        //End against the side wall of the hub
        frc::Pose2d(41_in, -4.140_in, frc::Rotation2d(-22.41_deg)),
        *config
    );

    config->SetReversed(true);
    auto Trajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(
        // get balls
        frc::Pose2d(41_in, -4.140_in, frc::Rotation2d(-22.41_deg)),
        {},
        //End against the side wall of the hub
        frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
        *config
    );

    std::cout << "shootpreload\n";
    frc2::RamseteCommand ramseteCommand(
        Trajectory1, [this]() { return m_drive->getPose(); },
        frc::RamseteController(kRamseteB, kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            K_S, K_V, K_A),
        K_DRIVE_KINEMATICS,
        [this] { return m_drive->getWheelSpeeds(); },
        frc2::PIDController(kP_, 0, 0),
        frc2::PIDController(kP_, 0, 0),
        [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
        {m_drive});

    std::cout << "returntostart\n";
    frc2::RamseteCommand ramseteCommand2(
        Trajectory2, [this]() { return m_drive->getPose(); },
        frc::RamseteController(kRamseteB, kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            K_S, K_V, K_A),\
        K_DRIVE_KINEMATICS,
        [this] { return m_drive->getWheelSpeeds(); },
        frc2::PIDController(kP_, 0, 0),
        frc2::PIDController(kP_, 0, 0),
        [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
        {m_drive});

    std::cout << "getballsandshoot\n";
    frc2::RamseteCommand ramseteCommand3(
        Trajectory3, [this]() { return m_drive->getPose(); },
        frc::RamseteController(kRamseteB, kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            K_S, K_V, K_A),
        K_DRIVE_KINEMATICS,
        [this] { return m_drive->getWheelSpeeds(); },
        frc2::PIDController(kP_, 0, 0),
        frc2::PIDController(kP_, 0, 0),
        [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
        {m_drive});

    frc2::RamseteCommand ramseteCommand4(
        Trajectory4, [this]() { return m_drive->getPose(); },
        frc::RamseteController(kRamseteB, kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            K_S, K_V, K_A),
        K_DRIVE_KINEMATICS,
        [this] { return m_drive->getWheelSpeeds(); },
        frc2::PIDController(kP_, 0, 0),
        frc2::PIDController(kP_, 0, 0),
        [this](auto left, auto right) { m_drive->tankDriveVolts(left, right); },
        {m_drive});

    // no auto
        std::cout << "vroom\n";
        return new frc2::SequentialCommandGroup(

        frc2::WaitCommand(0.5_s),
        
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { 
            m_drive->tankDriveVolts(0_V, 0_V);
            std::cout << "Stop1\n"; 
            }, {}),
        
        frc2::WaitCommand(0.5_s),
        
        std::move(ramseteCommand2),
        frc2::InstantCommand([this] { 
            m_drive->tankDriveVolts(0_V, 0_V);
            std::cout << "Stop2\n"; 
            }, {}),

        frc2::WaitCommand(0.5_s),
        
        std::move(ramseteCommand3),
        frc2::InstantCommand([this] { 
            m_drive->tankDriveVolts(0_V, 0_V);
            std::cout << "Stop3\n"; 
            }, {}),

        frc2::WaitCommand(0.5_s),
        
        std::move(ramseteCommand4),
        frc2::InstantCommand([this] { 
            m_drive->tankDriveVolts(0_V, 0_V);
            std::cout << "Stop3\n"; 
            }, {})
            
        );
}

void RobotContainer::InitAutoChoices() {
    std::cout << "initauto: step1\n";    
    m_chooser.SetDefaultOption(kAutoName_TestPath, "Test Path");
    m_chooser.AddOption("Robot", Bozo::CROSS_INIT_LINE);
    m_chooser.AddObject(kAutoName_CIL, "Cross Initial Line");
    m_chooser.AddObject(kAutoName_ShootPreload, "Shoot Preload");
}                  






























//congratulations! you found an easter egg!