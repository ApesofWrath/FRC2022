#include "RobotContainer.hpp"
#include <units/length.h>
#include <units/voltage.h>
#include <units/time.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

#include <iostream>
#include <vector>

const float K_TRACK_WIDTH = 0.688975;
constexpr auto K_MAX_ACCEL = 10.77;
constexpr auto K_S = units::volt_t(0.53229);
constexpr auto K_V = 2.8908 * 1_V * 1_s / 1_m;
constexpr auto K_A = 0.11875 * 1_V * 1_s * 1_s / 1_m;
constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
constexpr auto kRamseteZeta = 0.7 / 1_rad;

constexpr auto kP_ = 0.00019388;

constexpr auto kMaxForwardVelocity = 3.870452_m / 1_s;
// constexpr auto kMaxForwardVelocity = 1_m / 1_s;

const frc::DifferentialDriveKinematics K_DRIVE_KINEMATICS{
    units::meter_t(K_TRACK_WIDTH)};

frc2::Command* RobotContainer::GetAutonomousCommand() {

    frc::DifferentialDriveVoltageConstraint *autoVoltageConstraint = new frc::DifferentialDriveVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meter>(K_S, K_V, K_A),
        K_DRIVE_KINEMATICS, units::volt_t(10));

    // Set up config for trajectory
    frc::TrajectoryConfig *config = new frc::TrajectoryConfig(units::meters_per_second_t(2),
                                units::meters_per_second_squared_t(5));
    // Add kinematics to ensure max speed is actually obeyed
    config->SetKinematics(K_DRIVE_KINEMATICS);
    // Apply the voltage constraint
    config->AddConstraint(*autoVoltageConstraint);


    std::vector<frc::Translation2d> points = {
        frc::Translation2d(1_m,0_m),
        frc::Translation2d(0.667_m,1.5_m),
        frc::Translation2d(-1_m,1.5_m)
    };


    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        points,
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass the config
        *config);

    std::cout << "step 10\n";
    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive->getPose(); },
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
    std::cout << "step 11\n";
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { 
            m_drive->tankDriveVolts(0_V, 0_V);
            std::cout << "Stop\n"; 
            }, {}));
}

void RobotContainer::InitAutoChoices() {
    std::cout << "initauto: step1\n";
    m_chooser.SetDefaultOption(kAutoName_TestPath, "TestPath");    
    m_chooser.AddObject(kAutoName_UnnamedPath, "UnnamedPath");
    m_chooser.AddObject(kAutoName_Unnamed, "Unnamed");
}