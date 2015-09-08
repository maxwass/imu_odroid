#include "motor.h"
#include <stdlib.h>
#include <sys/ioctl.h>
#define MOTOR_PATH "/dev/i2c-4"
using namespace std;

// Member functions definitions including constructor
motor::motor(int which_motor, int comm_address)
{
    cout << "Motor object is being created, motor = " << which_motor << endl;
    motor_id = which_motor;
    i2c_address = comm_address;
    i2c_handle = open_i2c();
    
}
 
void motor::set_force( int force_in )
{ //when setting force, check that ...
    //the motors are allowed to run (CONTROLLER_RUN flag is true)
    //the force is within acceptable bounds
    if(CONTROLLER_RUN) { force = ensure_valid_force(force_in) };   //ENSURE this has access to CONTROLLER_RUN
    shut_down();
}
 
int motor::get_force( void )
{ return force; }

int motor::ensure_valid_force(int force_in)
{   //check if requested force of this motor is in the acceptable bounds
    // if not cap it at the max/min
    if(force_in > max_force) {return max_force;}
    if(force_in < min_force) {return min_force;}
    return force_in
}

void motor::shut_down(void)
{ force = 0;
   send_force_i2c();
 }

void motor::which_motor(void){
	return motor_id;
}

void motor::send_force_i2c(void){
    //Input/Output control: send to i2c_address
    ioctl(i2c_handle,I2C_SLAVE,i2c_address);

    //write(int fd, const void *buf, size_t count)
        //write() writes up to count bytes from the buffer 
        //pointed buf to the file referred to by the file 
        //descriptor fd.
    int success_write = write(i2c_handle, &force, 1);

    if(success_write < 0) {
        cout << "Failed to write to motor" << motor_id << endl;
    }

}

int open_i2c(void){
    cout << "opening i2c port...";
    int handle = open(MOTOR_PATH,O_RDWR);

    if (handle > 0) {cout << "Done!" << endl;}
    else            {cout << "Fail to open i2c port!" << endl;}

    return handle;
}

static int get_i2c(void){
    return i2c_handle;
}