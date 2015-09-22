//=================================
// include guard
#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

//=================================
// included dependencies
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
#include "motor.h"   //#include because motor_test contains a motor object
#include "imu.h"
#include "receiver.h"
#include <time.h>
//=================================
// forward declared dependencies
//struct State;

#define NUM_THREADS 4
#define XBEE_START_BYTE 0xBD
#define PI 3.14159265359


//=================================
//local data structures
 typedef struct desired_angles {
 double theta, phi, psi;
 } Desired_angles ;
 
 typedef struct control_command {
 double thrust, roll_acc, pitch_acc, yaw_acc;
 } Control_command;

 typedef struct gains {
 double kp_theta, kd_theta, kp_phi, kd_phi, kp_psi, kd_psi;
 } Gains; 




//function prototypes
void *command_input(void *thread_id);
void *control_stabilizer(void *thread_id);
void *buffer_thread(void *thread_id);
void *motor_signals(void *thread_id);
void init(void);
void start_motors(void);
void stop_motors(void);
void controller_on_off(void);
void display_on_off(void);
void set_Utrim(Control_command& U_trim);
void set_gains(Gains& gains);
void set_desired_angles(Desired_angles& desired_angles);
State state_error(const State& imu_data, const Desired_angles& desired_angles);
Control_command thrust(const State& error, const Control_command& U_trim, const Gains& gains);
void set_forces(const Control_command& U, double Ct, double d);
void display_info(const State& imu_data, const State& error, const Control_command& U);
void configure_threads(void);


#endif 
// __MOTOR_TEST_H_INCLUDED__
