//=================================
// include guard
#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#define VICON_MEM_LOC "VICON_MEM_LOC"
#define IMU_MEM_LOC   "IMU_MEM_LOC"

#include <sys/time.h>
#include <time.h> // for struct timeval
typedef struct state {
    float theta, phi, psi, theta_dot, phi_dot, psi_dot;
} State ;
 
typedef struct angles {
double theta, phi, psi;
} Angles ;

typedef struct vicon {
float x, y, z, phi, theta, psi;
} Vicon ;

typedef struct positions {
double x, y, z;
} Positions;
 
typedef struct control_command {
double thrust, roll_acc, pitch_acc, yaw_acc;
} Control_command;

typedef struct gains {
double kp_theta, kd_theta, kp_phi, kd_phi, kp_psi, kd_psi;
} Gains; 

typedef struct times {
timeval current, prev, prev_2, delta;
} Times; 



#endif
// __DATA_STRUCTS_H_INCLUDED__
