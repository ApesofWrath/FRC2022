    const double MAX_Y_RPM = 330.6005859; // placeholder 595.0;
    const double MAX_YAW_RATE = 9.8; // 11.4

    const double K_P_L = 0.00014286; // 0.0001558;
    const double K_P_R = 0.00014286; // 0.000103;
    const double K_D_L = 0;
    const double K_D_R = 0;
    const double K_P_YAW = 0.8;
    const double K_D_YAW = 0;

    const double MAX_Y_RPM_L_FORWARDS = 331.1132813; // 600.0;
    const double MAX_Y_RPM_L_BACKWARDS = 330.6152344; // 595.0;
    const double MAX_Y_RPM_R_FORWARDS = 330.6005859; // 590.0;
    const double MAX_Y_RPM_R_BACKWARDS = 331.2451172; // 595.0;

    const double SENSOR_TIMESTEP_MINUTE_CONVERSION = 600.0;
    const double TICKS_PER_ROT = 2048.0 * (63.0 / 5.0); // Values for 2020
    const double TICKS_TO_ROT = (63.0 / 5.0);

    constexpr double kOutputPercent = 1;