#include "motor.h"
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <sys/ioctl.h>
#define MOTOR_PATH "/dev/i2c-4"

int motor::open_i2c(void){
    cout << "opening i2c port...";
    int handle = open(MOTOR_PATH,O_RDWR);

    if (handle > 0) {cout << "Done!" << endl;}
    else            {cout << "Fail to open i2c port!" << endl;}

    return handle;
}

// Member functions definitions including constructor
motor::motor(int motor_id, int i2c_address)
{
    cout << "Motor object is being created, motor = " << motor_id << endl;
    this -> motor_id = motor_id;
    this -> i2c_address = i2c_address;
    i2c_handle = open_i2c();
    
}
 
void motor::set_force( int force_in , bool CONTROLLER_RUN)
{ //when setting force, check that ...
    //the motors are allowed to run (CONTROLLER_RUN flag is true)
    //the force is within acceptable bounds
    if(CONTROLLER_RUN) { force = ensure_valid_force(force_in) ;} 

}
 
uint8_t motor::get_force( void )
{ return force; }

int motor::ensure_valid_force(int force_in)
{   //check if requested force of this motor is in the acceptable bounds
    // if not cap it at the max/min
    if(force_in > max_force) {return max_force;}
    if(force_in < min_force) {return min_force;}
    return force_in;
}

void motor::shut_down(void)
{ force = 0;
   send_force_i2c();
 }

int motor::which_motor(void){
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


//int get_i2c(void){
//    return i2c_address;
//}
