#include <iostream>
#include <string>
#include <ctre/Phoenix.h>

class Climber {
    private:

        WPI_TalonFX *climber_talon1;
        WPI_TalonFX *climber_talon2;

    public:
    
    enum States {
        STOP_CLIMB, UP_CLIMB, DOWN_CLIMB, ZERO_CLIMB
    };

    States current_state;
    States last_state;

    Climber();

    void Stop();
    void Up();
    void Down();
    void Zero();
    void climberStateMachine();

    //inline void setState(States state) { }
};

