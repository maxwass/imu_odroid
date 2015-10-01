//=================================
// include guard
#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#define VICON_MEM_LOC "VICON_MEM_LOC"
#define IMU_MEM_LOC   "IMU_MEM_LOC"

typedef struct state {
    float theta, phi, psi, theta_dot, phi_dot, psi_dot;
} State ;
 
typedef struct angles {
double theta, phi, psi;
} Angles ;

typedef struct vicon {
double x, y, z, theta, phi, psi;
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
double t_current, t_,prev, t_prev_2, delta_t;
} Times; 



#endif
// __DATA_STRUCTS_H_INCLUDED__