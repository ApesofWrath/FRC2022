#pragma once

#include <units/length.h>
#include <units/time.h>
#include <units/angle.h>

const double MAX_Y_RPM = 595.0; // placeholder
const double MAX_YAW_RATE = 9.8; //11.4 -> 9.8

const double K_P_L = 0.00014286; //0.0001558
const double K_P_R = 0.00014286; //0.000103
const double K_D_L = 0;
const double K_D_R = 0;
const double K_P_YAW = 0.8;
const double K_D_YAW = 0;

const double MAX_Y_RPM_L_FORWARDS = 600.0;
const double MAX_Y_RPM_L_BACKWARDS = 595.0;
const double MAX_Y_RPM_R_FORWARDS = 590.0;
const double MAX_Y_RPM_R_BACKWARDS = 595.0;


constexpr double kTicksPerRot = 2048;
constexpr auto kCycleTime = 0.1;
constexpr auto kWheelDiameter = 6;
constexpr auto kWheelDiameter_m = kWheelDiameter * 0.0254;
constexpr auto kWheelCirc = kWheelDiameter_m * 3.14159;

constexpr auto kTicksPerMeter = 2048.0 * (63.0 / 5.0) / kWheelCirc;
constexpr auto kMetersPerTick = 1.0 / kTicksPerMeter;
/*

(ticks/motor rot)(motor rot/wheel rot)/(wheel circ meters)



*/


const auto SENSOR_TIMESTEP_MINUTE_CONVERSION = 600.0;
const auto TICKS_PER_ROT = 2048.0 * (63.0 / 5.0);
const auto TICKS_TO_ROT = (63.0 / 5.0);
const auto WHEEL_DIAMETER = 6;
const auto WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * 3.14159;
const auto TICKS_PER_INCH = WHEEL_CIRCUMFRENCE * TICKS_PER_ROT;
const auto SENSOR_TIMESTEP_SECOND_CONVERSION = 10.0;

constexpr double kOutputPercent = 1;