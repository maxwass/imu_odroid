#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions, Unix API for terminal I/O */


#define BAUDRATE B57600
#define MY_PATH "/dev/ttySAC0"

typedef struct state {
    float theta, phi, psi, theta_dot, phi_dot, psi_dot;
} Sstate ;

 typedef struct desired {
 double theta, phi, psi;
 } Sdesired ;
 
 typedef struct control_command {
 double thrust, roll_acc, pitch_acc, yaw_acc;
 } Scontrol_command;
 
 typedef struct forces {
 double motor_1, motor_2, motor_3, motor_4;
 } Sforces;
 
using namespace std;

//interface
int open_port(void);
void print_data(const Sstate& imu_data);
void unpack_data(Sstate& imu_data, const unsigned char arr[]);
void get_data(const int port, Sstate& imu_data);
