#include <Hood.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

// use buttons 4 and 5 for hood adjustment

Hood::Hood() {
    h_Motor = std::make_shared<TalonFX>(0);
}