#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include "imu.h"
#include "receiver.h"
#include <math.h>
#include <stdint.h>

#define NUM_THREADS 3
#define XBEE_START_BYTE 0xBD
#define PI 3.14159265359

pthread_mutex_t data_acq_mutex; 
pthread_cond_t data_acq_cv;

struct timeval t_init, t_now, t_now_v;
double t_v, del_t_v, t_prev_v = 0.0;
bool SYSTEM_RUN = true;
bool CONTROLLER_RUN = false;
bool DISPLAY_RUN = true;
 int i2cHandle, usb_imu, usb_xbee, res1;
 int motor_out[4] = {30, 30, 30, 30};
 int F_init = 30;
float phi_d = 0.0;
float theta_d = 0.0;\
float psi_d = 0.0;
float e_phi, e_phi_dot, e_theta, e_theta_dot, e_psi, e_psi_dot;
int U[4], U_trim[4], ff[4];
float psi_init;

float kp_phi = 5.5;
float kd_phi = 0.32;

float kp_theta = 5.5;
float kd_theta = 0.32;

float kp_psi = 5.2;
float kd_psi = 0.3;



void *command_input(void *thread_id)
{
 unsigned char buf[2]; // command from the host computer
    
    while(SYSTEM_RUN == true) {

	tcflush(usb_xbee,TCIOFLUSH);

	read(usb_xbee,buf,2);

	if (buf[0] == 0xBD)
		printf("recieved: %c\n",buf[1]);
	else
		printf("Wrong start byte!\n");
		//printf("anyway recieved: %c\n",buf[1]);
	tcflush(usb_xbee,TCIOFLUSH);

	if (buf[1] == '1')
	{printf("Motor 1 is running...\n");
	motor_out[0] = 30;
	motor_out[1] = 0;
	motor_out[2] = 0;
	motor_out[3] = 0;
	}
	else if (buf[1] == '2')
	{printf("Motor 2 is running...\n");
	motor_out[0] = 0;
	motor_out[1] = 30;
	motor_out[2] = 0;
	motor_out[3] = 0;
	}
	else if (buf[1] == '3')
	{printf("Motor 3 is running...\n");
	motor_out[0] = 0;
	motor_out[1] = 0;
	motor_out[2] = 30;
	motor_out[3] = 0;
	}
	else if (buf[1] == '4')
	{printf("Motor 4 is running...\n");
	motor_out[0] = 0;
	motor_out[1] = 0;
	motor_out[2] = 0;
	motor_out[3] = 30;
	}
	else if (buf[1] == '5' || buf[1] == 'q' || buf[1] == 'Q')
	{printf("Motor Stop!\n");
	motor_out[0] = 0;
	motor_out[1] = 0;
	motor_out[2] = 0;
	motor_out[3] = 0;
	CONTROLLER_RUN = false;
	}

	else if (buf[1] == 't' || buf[1] == 'T')
	{
	if(CONTROLLER_RUN == false)
	{printf("Controller ON!!\n");
	CONTROLLER_RUN = true;}
	else if(CONTROLLER_RUN == true)
	{printf("Controller OFF!!\n");
	CONTROLLER_RUN = false;}
	}

	else if (buf[1] == 'p' || buf[1] == 'P')
	{
	if(DISPLAY_RUN == false)
	{DISPLAY_RUN = true;}
	else if(DISPLAY_RUN == true)
	{DISPLAY_RUN = false;}
	}

	else if (buf[1] == 'r' || buf[1] == 'R')
	{printf("Increase Thrust\n");
	U_trim[0] = U_trim[0] + 4;
	}
	else if (buf[1] == 'f' || buf[1] == 'F')
	{printf("Decrease Thrust!\n");
	U_trim[0] = U_trim[0] - 4;
	}

	else if (buf[1] == 'w' || buf[1] == 'W')
	{printf("Increase kp\n");
	kp_phi = kp_phi + 0.1;
	kp_theta = kp_theta + 0.1;
	}
	else if (buf[1] == 's' || buf[1] == 'S')
	{printf("Decrease kp!\n");
	kp_phi = kp_phi - 0.1;
	kp_theta = kp_theta - 0.1;
	}

	else if (buf[1] == 'e' || buf[1] == 'E')
	{printf("Increase kd\n");
	kd_phi = kd_phi + 0.03;
	kd_theta = kd_theta + 0.03;
	}
	else if (buf[1] == 'd' || buf[1] == 'D')
	{printf("Decrease kd\n");
	kd_phi = kd_phi - 0.03;
	kd_theta = kd_theta - 0.03;
	}
	
	else if (buf[1] == 'i' || buf[1] == 'I')
	{printf("decrease phi_d\n");
	theta_d = theta_d - 1.0;
	}
	else if (buf[1] == 'k' || buf[1] == 'K')
	{printf("Increase phi_d\n");
	theta_d = theta_d + 1.0;
	}
	else if (buf[1] == 'j' || buf[1] == 'J')
	{printf("decrease theta_d\n");
	phi_d = phi_d - 1.0;
	}
	else if (buf[1] == 'l' || buf[1] == 'L')
	{printf("Increase theta_d\n");
	phi_d = phi_d + 1.0;
	}

    }
    pthread_exit(NULL);
}






void *control_stabilizer(void *thread_id)
{

 int i;
 int address[4] = {0x2c, 0x29, 0x2b, 0x2a}; //The address of i2c
 int res_mtr_out, motorspeed_lowerbits;
 uint8_t motorspeed[2] = { 0 };
 unsigned char sensor_bytes2[24];
 double Ct=0.013257116418667*10;
 double d=0.169;
 int start_motor = 0x10;
    
    while(SYSTEM_RUN == true) {

	SImu_data imu_data;
	tcflush(usb_imu, TCIFLUSH);
	res1 = read(usb_imu,&sensor_bytes2[0],24);

	imu_data.theta = *(float *)&sensor_bytes2[4];
	imu_data.phi = *(float *)&sensor_bytes2[8];
	imu_data.psi = *(float *)&sensor_bytes2[0];
	imu_data.phi_dot = *(float *)&sensor_bytes2[12];
	imu_data.theta_dot = *(float *)&sensor_bytes2[16];
	imu_data.psi_dot = *(float *)&sensor_bytes2[20];
 
	if(CONTROLLER_RUN == false)
	{
	psi_init  =  imu_data.psi;
	motorspeed[0] = 0 ;
	res_mtr_out = write(i2cHandle, &motorspeed,1);
}
	imu_data.psi = imu_data.psi - psi_init;

	//printf("phi, theta, psi, phi_dot, theta_dot, psi_dot: %E, %E, %E, %E, %E, %E \n", imu_data.phi, imu_data.theta, imu_data.psi, imu_data.phi_dot, imu_data.theta_dot, imu_data.psi_dot);
	tcflush(usb_imu, TCIFLUSH);

	e_phi = (-imu_data.phi + phi_d) * PI/180;
	e_theta = (-imu_data.theta + theta_d) * PI/180;
	e_psi = (-imu_data.psi + psi_d) * PI/180;
	e_phi_dot = (-imu_data.phi_dot) * PI/180;
	e_theta_dot = (-imu_data.theta_dot) * PI/180;
	e_psi_dot = (-imu_data.psi_dot) * PI/180;

	U[0] = 30 + U_trim[0];
	U[1] = kp_phi * e_phi + kd_phi * e_phi_dot + U_trim[1];
	U[2] = kp_theta * e_theta + kd_theta * e_theta_dot + U_trim[2];
	U[3] = kp_psi * e_psi + kd_psi * e_psi_dot + U_trim[3];

	
    
    	 //printf("Moment: %E %E %E\n",M[0],M[1],M[2]);
   	 //Calculating the forces of each motor
    
    	ff[0] = (U[0]/4-(U[3]/(4*Ct))+(U[2]/(2*d)));
   	ff[1] = (U[0]/4+(U[3]/(4*Ct))-(U[1]/(2*d)));
   	ff[2] = (U[0]/4-(U[3]/(4*Ct))-(U[2]/(2*d)));
   	ff[3] = (U[0]/4+(U[3]/(4*Ct))+(U[1]/(2*d)));

   	int i;

   	double a[4]={3.0107,    3.0107,    3.0107,    3.0107};
   	double b[4]={1.4400,    1.4400,    1.4400,    1.4400};
   	double c[4]={0.0000,    0.0000,    0.0000,    0.0000};
	
	
	
    
	
	for(i=0;i<4;i++)
		{
		if(ff[i] < 0.0) ff[i] = 0.0;

		if(CONTROLLER_RUN == true)
    		{
			//motor_out[i]=ceil((-b[i]+sqrt(b[i]*b[i]-4.*a[i]*(c[i]-ff[i])))/2./a[i]*240.);
			motor_out[i] = ff[i];
		}
		else
			motor_out[i] = 0.0;

		if(motor_out[i] > 2047.0) motor_out[i]=2047.0;
		else if(motor_out[i] < 0.0)	motor_out[i]=0.0;
		}	

//(uint8_t)(tmpVal >> 3) & 0xff
	for(i=2;i<3;i++)
	{
	 ioctl(i2cHandle,I2C_SLAVE,address[i]);
	 //motorspeed = (int)motor_out[i] / 8;
	 motorspeed[0] = (uint8_t)(motor_out[i] >> 3) & 0xff;
	// motorspeed_lowerbits = (((int)motor_out[i] % 8) & 0x07);
	motorspeed[1] = (((uint8_t)motor_out[i] % 8) & 0x07);

	 res_mtr_out = write(i2cHandle, &motorspeed,2);

	 //res_mtr_out = write(i2cHandle, &motorspeed_lowerbits,1);

	 printf("motor out: %d, %d\n", motorspeed[0], motorspeed[1]);
	}

	if(DISPLAY_RUN == true)
	{
/*		printf("<==========================================>\n");
		if(CONTROLLER_RUN == true) printf("Controller ON \n");
		else if (CONTROLLER_RUN == false) printf("Controller OFF \n");
		printf("	IMU DATA	\n");
		printf("phi: %.2f         phi dot: %.2f\n",imu_data.phi, imu_data.phi_dot);
		printf("theta: %.2f         theta dot: %.2f\n",imu_data.theta, imu_data.theta_dot);
		printf("psi: %.2f         psi dot: %.2f\n\n\n",imu_data.psi, imu_data.psi_dot);

		printf("	GAINS		\n");
		printf("kp_phi: %f	kd_phi: %f\n",kp_phi, kd_phi);
		printf("kp_theta: %f	kd_theta: %f\n",kp_theta, kd_theta);
		printf("kp_psi: %f	kd_psi: %f\n\n\n",kp_psi, kd_psi);

		printf("	MOTOR_OUT	\n");
		printf("Motor #1: %d,      raw: %d\n", motor_out[0], ff[0]);
		printf("Motor #2: %d,      raw: %d\n", motor_out[1], ff[1]);		
		printf("Motor #3: %d,      raw: %d\n", motor_out[2], ff[2]);
		printf("Motor #4: %d,      raw: %d\n\n\n", motor_out[3], ff[3]);
		
		printf("	Errors		\n");
		printf("e_phi: %f,	e_theta: %f,	e_psi: %f\n\n\n", e_phi*180/PI, e_theta*180/PI, e_psi*180/PI); */
		printf("Motor out: %d\n", motor_out[0]);
	}

    }
    pthread_exit(NULL);
}

void *buffer_thread(void *thread_id)
{
    
    while(SYSTEM_RUN == true) {

    }
    pthread_exit(NULL);
}



int main(void)
{

 pthread_t threads[3];
 pthread_attr_t attr;
 struct sched_param	param;
 int fifo_max_prio, fifo_min_prio;

 system("clear");
 gettimeofday(&t_init,NULL);
   
 printf("opening i2c port...\n");
 i2cHandle = open("/dev/i2c-4",O_RDWR);

	if (i2cHandle > 0)
		printf("Done!\n");
	else
		printf("Fail to open i2c port!\n");
 usleep(10000);

 printf("Opening an USB port...   ");//Opens the usb Port
 usb_xbee = open_usbport();
	if (usb_xbee <0)
        	printf("\n Error opening an USB0 port!!\n");
        else
        	printf("Done!\n");
 usleep(10000);

 printf("opening usb port for imu...\n");
 usb_imu = open_port();
	if (usb_imu > 0)
		printf("Done!\n");
	else
		printf("Fail to open usb port!\n");
 usleep(100000);

  // Initialize mutex and condition variables
    pthread_mutex_init(&data_acq_mutex, NULL);
    
    // Set thread attributes
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
    fifo_min_prio = sched_get_priority_min(SCHED_FIFO);
    
    // Create threads
    // Higher priority for filter
    param.sched_priority = fifo_max_prio;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&threads[0], &attr, control_stabilizer, (void *) 0);
    
    // Medium priority for vicon
    param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&threads[1], &attr, buffer_thread, (void *) 1);
    
    // Lower priority for vicon
    param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&threads[2], &attr, command_input, (void *) 2);
    
    // Wait for all threads to complete
    for (int i = 0; i < NUM_THREADS; i++)
    {
        pthread_join(threads[i], NULL);
    }
    
    close(i2cHandle);
    close(usb_xbee);
    close(usb_imu);
    
    pthread_attr_destroy(&attr);
    pthread_mutex_destroy(&data_acq_mutex);
    return 0;
    

}

