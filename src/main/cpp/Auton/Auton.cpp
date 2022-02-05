#include "Auton/Auton.hpp"
#include <units/length.h>

AutonDrive::AutonDrive(int left1, int left2, int right1, int right2, AHRS* ahrs_)  : 
        left1_drive{left1}, left2_drive{left2}, right1_drive{right1}, right2_drive{right2}, ahrs{ahrs_} {

    power_distribution = new frc::PowerDistribution(0, frc::PowerDistribution::ModuleType::kCTRE);

    left1_drive.ConfigFactoryDefault();
    left2_drive.ConfigFactoryDefault();
    right1_drive.ConfigFactoryDefault();
    right2_drive.ConfigFactoryDefault();

    left1_drive.SetSafetyEnabled(false);
    left2_drive.SetSafetyEnabled(false);
    right1_drive.SetSafetyEnabled(false);
    right2_drive.SetSafetyEnabled(false);
    
    left1_drive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    right1_drive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    
    left1_drive.SetNeutralMode(NeutralMode::Brake);
    left2_drive.SetNeutralMode(NeutralMode::Brake);
    right1_drive.SetNeutralMode(NeutralMode::Brake);
    right2_drive.SetNeutralMode(NeutralMode::Brake);

    left1_drive.SetInverted(true);
    left2_drive.SetInverted(true);

    odometry = new frc::DifferentialDriveOdometry(frc::Rotation2d(units::degree_t(getHeading())));

    drive.SetSafetyEnabled(false);
}

void AutonDrive::Periodic() {
    getWheelSpeeds();
    odometry->Update(frc::Rotation2d(units::degree_t(getHeading())),
        units::meter_t(left1_drive.GetSelectedSensorPosition() * kMetersPerTick),
        units::meter_t(right1_drive.GetSelectedSensorPosition() * kMetersPerTick)
    );    
}

void AutonDrive::tankDriveVolts(units::volt_t left, units::volt_t right) {
    left_motors.SetVoltage(left);
    right_motors.SetVoltage(right);

    frc::SmartDashboard::PutNumber("v left", left());
    frc::SmartDashboard::PutNumber("v right", right());
    drive.Feed();
}

double AutonDrive::getAverageEncoderDistance() {
    return (left1_drive.GetSelectedSensorPosition() * kMetersPerTick);
}

void AutonDrive::setMaxOutput(double maxOutput) {
    drive.SetMaxOutput(maxOutput);
}

double AutonDrive::getHeading() {
    return std::remainder(-ahrs->GetAngle(), 360);
}

double AutonDrive::getTurnRate() {
    return ahrs->GetRate();
}

frc::Pose2d AutonDrive::getPose() { 
    return odometry->GetPose();
}
// hi friends :)
//hola senors ;
frc::DifferentialDriveWheelSpeeds AutonDrive::getWheelSpeeds() {
    return {units::meters_per_second_t(left1_drive.GetSelectedSensorVelocity() * kMetersPerTick * 10.0),
          units::meters_per_second_t(right1_drive.GetSelectedSensorVelocity() * kMetersPerTick * 10.0)};
}

void AutonDrive::resetOdometry(frc::Pose2d pose) {
  resetEncoders();
  ahrs->Reset();
  ahrs->ZeroYaw();
  odometry->ResetPosition(pose,frc::Rotation2d(units::degree_t(0)));
}

void AutonDrive::resetEncoders() {
    left1_drive.SetSelectedSensorPosition(0);
    right1_drive.SetSelectedSensorPosition(0);
}
