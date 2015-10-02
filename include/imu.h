//=================================
// include guard
#ifndef IMU_H
#define IMU_H

//=================================
// included dependenciesa
#include "data_structs.h"

#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions, Unix API for terminal I/O */

#define BAUDRATE B115200
#define MY_PATH "/dev/ttySAC0"

using namespace std;

//function prototypes
int open_port(void);
void print_data(const State& imu_data);
void unpack_data(State& imu_data, const unsigned char arr[]);
void get_data(const int port, State& imu_data);


#endif
// __IMU_H_INCLUDED__
