//=================================
// include guard
#ifndef CONTROLLER_H
#define CONTROLLER_H

//=================================
// included dependencies
#include "motor.h"   //#include because motor_test contains a motor object
#include "imu.h"
#include "data_structs.h" // user defined structs (state, control_command,gains, desired_angles)
#include "concur_data.h"


#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "receiver.h"
#include <time.h>

//=================================
// forward declared dependencies

//


#define NUM_THREADS 3
#define XBEE_START_BYTE 0xBD
#define PI 3.14159265359


//=================================




//function prototypes
void *command_input(void *thread_id);
void *control_stabilizer(void *thread_id);
void *motor_signals(void *thread_id);
void init(void);
void start_motors(void);
void stop_motors(void);
void controller_on_off(bool& CONTROLLER_RUN);
void display_on_off(bool& DISPLAY_RUN);
void set_Utrim(Control_command& U_trim);
void set_gains(Gains& gains);
void set_desired_angles(Angles& desired_angles);
State state_error(const State& imu_data, const Angles& desired_angles);
Control_command thrust(const State& error, const Control_command& U_trim, const Gains& gains);
void set_forces(const Control_command& U, double Ct, double d);
void display_info(const State& imu_data, const State& error, const Control_command& U);
void configure_threads(void);


#endif 
// __CONTROLLER_H_INCLUDED__
