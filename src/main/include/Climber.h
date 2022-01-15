#include <string>
#include <ctre/Phoenix.h>

class Climber {
    private:
    
        WPI_TalonFX *climber_talon1;
        WPI_TalonFX *climber_talon2;

    public:
    
    enum States {
        STOP_CLIMB, UP_CLIMB, DOWN_CLIMB
    };

    States current_state;
    States last_state;

    Climber();

    void Stop();
    void Up();
    void Down();
    void StateMachine();

    const double idle_speed = 0.0;
    const double up_speed = 0.3;
    const double down_speed = -0.3;

};

