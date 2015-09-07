#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

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
#include <stdint.h>
//
#include "imu.h"

#define NUM_THREADS 3
#define XBEE_START_BYTE 0xBD
#define PI 3.14159265359

//user defines data structures
 typedef struct desired {
 double theta, phi, psi;
 } Sdesired ;
 
 typedef struct control_command {
 double thrust, roll_acc, pitch_acc, yaw_acc;
 } Scontrol_command;
 
 typedef struct forces {
 double motor_1, motor_2, motor_3, motor_4;
 } Sforces;


//function prototypes
void *command_input(void *thread_id);
void *control_stabilizer(void *thread_id);
void *buffer_thread(void *thread_id);
void start_motors(void);
void stop_motors(void);
void controller_on_off(bool CONTROLLER_RUN);
void display_on_off(bool DISPLAY_RUN);
void set_Utrim(Scontrol_command& U_trim);
void set_gains(Sstate& gains);
void set_desired_angles(Sdesired& desired_angles);
Sstate state_error(const Sstate& imu_date, const float phi_d, const float theta_d);
Scontrol_command thrust(const Sstate& error, const int U_trim[], const Sgains& gains);
Sforces forces(const Scontrol_command& U, double Ct, double d);


#endif