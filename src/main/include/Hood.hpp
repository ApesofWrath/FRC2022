#pragma once

#include <ctre/Phoenix.h>
#include <memory>

constexpr float hoodSpeed = .1;
constexpr int hoodMotor = 1;


class Hood {
public:

    Hood();

private:

    std::shared_ptr<TalonFX> h_Motor;
};

