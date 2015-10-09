//=================================
// include guard
#ifndef CONTROLLER_H
#define CONTROLLER_H

//=================================
// included dependencies
#include "motor.h"   //#include because controller contains a motor object
#include "imu.h"
#include "data_structs.h" // user defined structs (state, control_command,gains, desired_angles)
#include "vicon.h" //DELETE THIS ONCE INTERPROCESS WORKS
//#include "shared_data.hpp" //UNCOMMENT WHEN IMPLEMENTED


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
//#include "receiver.h"
#include <time.h>
#include <sys/time.h>
#include <curses.h>

#define NUM_THREADS 3
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
void set_initial_times(Times& times);
void set_initial_positions(Positions& init_positions);
void set_timeval(timeval& x, timeval& y);
void time_calc(Times& times);
int timeval_subtract(timeval* result,timeval* x,timeval* y);
Angles angles(const Vicon& vicon_data, const Positions& desired_positions);
State state_error(const State& imu_data, const Angles& desired_angles);
Control_command thrust(const State& error, const Control_command& U_trim, const Gains& gains);
void set_forces(const Control_command& U, double Ct, double d);
Vicon vicon_velocity(Vicon& current, Vicon& old);
void display_info(const State& imu_data, const State& error, const Control_command& U, const Vicon& vicon_data);
void configure_threads(void);


#endif 
// __CONTROLLER_H_INCLUDED__
