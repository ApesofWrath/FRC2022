    const double MAX_Y_RPM = 595.0; // placeholder
    const double MAX_YAW_RATE = 11.4; //placeholder

    const double K_P_L = 0.0001558;
    const double K_P_R = 0.000103;
    const double K_D_L = 0;
    const double K_D_R = 0;
    const double K_P_YAW = 0.08;
    const double K_D_YAW = 0;

    const double MAX_Y_RPM_L_FORWARDS = 600.0;
    const double MAX_Y_RPM_L_BACKWARDS = 595.0;
    const double MAX_Y_RPM_R_FORWARDS = 590.0;
    const double MAX_Y_RPM_R_BACKWARDS = 595.0;

    const double SENSOR_TIMESTEP_MINUTE_CONVERSION = 600.0;
    const double TICKS_PER_ROT = 2048.0 * (63.0 / 5.0); // Values for 2020
    const double TICKS_TO_ROT = (63.0 / 5.0);