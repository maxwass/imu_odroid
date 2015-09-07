#include "motor.h"

// Member functions definitions including constructor
motor::motor(int which_motor)
{
    cout << "Motor object is being created, motor = " << which_motor << endl;
    motor_id = which_motor;
}
 
void motor::set_force( double force_in )
{ //when setting force, check that ...
    //the motors are allowed to run (CONTROLLER_RUN flag is true)
    //the force is within acceptable bounds
    if(CONTROLLER_RUN) { force = ensure_valid_force(force_in) };   //ENSURE this has access to CONTROLLER_RUN
    shut_down();
}
 
double motor::get_force( void )
{ return force; }

double motor::ensure_valid_force(double force_in)
{   //check if requested force of this motor is in the acceptable bounds
    // if not cap it at the max/min
    if(force_in > max_force) {return max_force;}
    if(force_in < min_force) {return min_force;}
    return force_in
}

void motor::shut_down(void)
{
    force = 0.0;
}

void motor::which_motor(void){
	return motor_id;
}
