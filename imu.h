#ifndef IMU_H
#define IMU_H

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
 
using namespace std;

//interface
int open_port(void);
void print_data(const Sstate& imu_data);
void unpack_data(Sstate& imu_data, const unsigned char arr[]);
void get_data(const int port, Sstate& imu_data);


#endif