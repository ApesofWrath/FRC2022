#include "Auton/Auton.hpp"
#include <units/length.h>

AutonDrive::AutonDrive(int left1, int left2, int right1, int right2, AHRS* ahrs_)  : 
        left1_drive{left1}, left2_drive{left2}, right1_drive{right1}, right2_drive{right2}, ahrs{ahrs_} {

    power_distribution = new frc::PowerDistribution(0, frc::PowerDistribution::ModuleType::kCTRE);

    configure();

    odometry = new frc::DifferentialDriveOdometry(frc::Rotation2d(units::degree_t(getHeading())));

    drive.SetSafetyEnabled(false);
}

void AutonDrive::configure() {
    left1_drive.ConfigFactoryDefault();
    left2_drive.ConfigFactoryDefault();
    right1_drive.ConfigFactoryDefault();
    right2_drive.ConfigFactoryDefault();

    left1_drive.SetSafetyEnabled(false);
    left2_drive.SetSafetyEnabled(false);
    right1_drive.SetSafetyEnabled(false);
    right2_drive.SetSafetyEnabled(false);
    
    // right2_drive.SetControlFramePeriod(ControlFrame::Control_3_General, 255);
    // right2_drive.SetControlFramePeriod(ControlFrame::Control_4_Advanced, 255);

    right2_drive.SetStatusFramePeriod(StatusFrame::Status_1_General_, 255);
    right2_drive.SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 255);

    // left2_drive.SetControlFramePeriod(ControlFrame::Control_3_General, 255);
    // left2_drive.SetControlFramePeriod(ControlFrame::Control_4_Advanced, 255);

    left2_drive.SetStatusFramePeriod(StatusFrame::Status_1_General_, 255);
    left2_drive.SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 255);

    left1_drive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    right1_drive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    
    left1_drive.SetNeutralMode(NeutralMode::Brake);
    left2_drive.SetNeutralMode(NeutralMode::Brake);
    right1_drive.SetNeutralMode(NeutralMode::Brake);
    right2_drive.SetNeutralMode(NeutralMode::Brake);

    // left1_drive.SetInverted(true);
    // left2_drive.SetInverted(true);
}

void AutonDrive::Periodic() {
    auto wspeeds = getWheelSpeeds();
    // frc::SmartDashboard::PutNumber("LeftPos", left1_drive.GetSelectedSensorPosition() * kMetersPerTick);
    // frc::SmartDashboard::PutNumber("RightPos", right1_drive.GetSelectedSensorPosition() * kMetersPerTick);

    odometry->Update(frc::Rotation2d(units::degree_t(getHeading())),
        units::meter_t(left1_drive.GetSelectedSensorPosition() * kMetersPerTick),
        units::meter_t(right1_drive.GetSelectedSensorPosition() * kMetersPerTick)
    );    
}

void AutonDrive::tankDriveVolts(units::volt_t left, units::volt_t right) {
    left_motors.SetVoltage(left);
    right_motors.SetVoltage(right);

    // frc::SmartDashboard::PutNumber("v left", left.value());
    // frc::SmartDashboard::PutNumber("v right", right.value());
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
    return -1.0 * ahrs->GetRate();
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
