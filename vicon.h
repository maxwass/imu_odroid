//=================================
// include guard
#ifndef VICON_H
#define VICON_H

//=================================
// included dependencie
// #include "imu.h"
// #include "motor.h"
#include "data_structs.h" // user defined structs (state, control_command,gains, desired_angles)
#include "concur_data.h"  //process communcation class
//
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
   #include <linux/i2c.h>
   #include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include "receiver.h"
#include <math.h>
   #include "serial1.h"
   #include "Xbee.h"

#include <curses.h> //for getch()

//=================================
// forward declared dependencies
//struct State;
#define NUM_THREADS 4
//#define XBEE_START_BYTE 0xBD
#define PI 3.14159265359


#define LIDAR_LITE_ADRS 0x62
#define MEASURE_VAL 0x04
#define MEASURE_REG 0x00
#define STATUS_REG  0x47
#define DISTANCE_REG_HI 0x0f
#define DISTANCE_REG_LO 0x10  
#define VERSION_REG 0x41
#define ERROR_READ -1
// Status Bits
#define STAT_BUSY               0x01
#define STAT_REF_OVER           0x02
#define STAT_SIG_OVER           0x04
#define STAT_PIN                0x08
#define STAT_SECOND_PEAK        0x10
#define STAT_TIME               0x20
#define STAT_INVALID            0x40
#define STAT_EYE                0x80
#define DISTANCE_CONT		    0x8f

void *vicon_input(void *thread_id);
void get_data(const int port, Vicon& vicon_data);
void unpack_data(Vicon& vicon_data, const unsigned char arr[]);
void time_calc(Times& times);
void set_desired_positions(Positions& desired_positions);
void set_start(Positions& START);
void set_initial_times(Times& times);
void configure_threads(void);
void display_info(const Vicon& vicon_data);
void init(void);
int main(void);



#endif 
// __VICON_H_INCLUDED__