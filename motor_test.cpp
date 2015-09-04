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
float theta_d = 0.0;
float psi_d = 0.0;
float e_phi, e_phi_dot, e_theta, e_theta_dot, e_psi, e_psi_dot;
int U[4], U_trim[4], ff[4];
float psi_init;
double Ct=0.013257116418667*10;
double d=0.169;

float kp_phi = 5.5;
float kd_phi = 0.32;

float kp_theta = 5.5;
float kd_theta = 0.32;

float kp_psi = 5.2;
float kd_psi = 0.3;

typedef struct gains {
    float kp_theta, kp_phi, kp_psi, kd_theta, kd_phi, kd_psi;
} Sgains ;

Sgains gains;
gains.kp_theta = 5.5;
gains.kd_theta = 0.32;

gains.kp_phi = 5.5;
gains.kd_phi = 0.32;

gains.kp_psi = 5.2;
gains.kd_psi = 0.3;


//executes input from host computer on motors, controller gains, displays, and controller
void *command_input(void *thread_id)
{
 //command from the host computer
 unsigned char buf[2];
    
    while(SYSTEM_RUN == true) {
        
    //discard data written to the object referred to by the file descriptor ("usb_xbee), TCIOFLUSH - flushes both
        //data received but not read, and data written but not transmitted.
	tcflush(usb_xbee,TCIOFLUSH);
        
    //read 2 bytes from file descriptor "usb_xbee" into buffer starting at "buf" which in this case is the command
        //from host computer
	read(usb_xbee,buf,2);

        if (buf[0] == 0xBD){printf("recieved: %c\n",buf[1]);}
        else{printf("Wrong start byte!\n");}
    
    //Why are we flushing HERE???
	tcflush(usb_xbee,TCIOFLUSH);

    //inputs map from buf[1] to controls: if buf[1] == ...
        //1-5       ==> motors ON/OFF
        //t/T       ==> controller ON/OFF
        //p/P       ==> display ON/OFF
        //r/R, f/F  ==> Inc/Dec Thrust
    //Phi/Theta
        //w/w, s/S ==> Inc/Dec Kp
        //e/E, d/D ==> Inc/Dec Kd
    //Theta
        //k/K, i/I ==> Inc/Dec theta_desired
    //Phi
        //l/L, j/J ==> Inc/Dec phi_desired
        
        unsigned int command = buf[1];
        
        switch (command) {
            case '1':
            case '2':
            case '3':
            case '4':
                start_motor(motor_out, command);
                break;
                
            case '5':
            case 'q':
            case 'Q':
                printf("Motor Stop!\n");
                for(int i = 0; i<4; i++) { motor_out[i] = 0; }
                CONTROLLER_RUN = false;
                break;
                
            case 't':
            case 'T':
                controller_on_off(CONTROLLER_RUN);
                break;
                
            case 'p':
            case 'P':
                display_on_off(DISPLAY_RUN);
                break;
                
            case 'r':
            case 'R':
                printf("Increase Thrust\n");
                U_trim[0] = U_trim[0] + 4;
                break;
                
            case 'f':
            case 'F':
                printf("Decrease Thrust!\n");
                U_trim[0] = U_trim[0] - 4;
                break;
                
            case 'w':
            case 'W':
                printf("Increase kp_phi and kp_theta\n");
                kp_phi = kp_phi + 0.1;
                kp_theta = kp_theta + 0.1;
                break;
                
            case 's':
            case 'S':
                printf("Decrease kp_phi and kp_theta\n");
                kp_phi = kp_phi - 0.1;
                kp_theta = kp_theta - 0.1;
                break;
                
            case 'e':
            case 'E':
                printf("Increase kd_phi and kd_theta\n");
                kd_phi = kd_phi + 0.03;
                kd_theta = kd_theta + 0.03;
                break;
                
            case 'd':
            case 'D':
                printf("Decrease kd_phi and kd_theta\n");
                kd_phi = kd_phi - 0.03;
                kd_theta = kd_theta - 0.03;
                break;
                
            case 'k':
            case 'K':
                printf("Increase theta_d ('theta desired') \n");
                theta_d = theta_d + 1.0;
                break;
                
            case 'i':
            case 'I':
                printf("Decrease theta_d ('theta desired') \n");
                theta_d = theta_d - 1.0;
                break;
                
            case 'l':
            case 'L':
                printf("Increase phi_d ('phi desired') \n");
                phi_d = phi_d + 1.0;
                break;
                
            case 'j':
            case 'J':
                printf("Decrease phi_d ('phi desired') \n");
                phi_d = phi_d - 1.0;
                break;
                
            default:
                printf("Command Input: Unrecognized input command");
                cout << command << endl;
                break;
        }

    }
    pthread_exit(NULL);
}


void *control_stabilizer(void *thread_id)
{

 int i;
 int address[4] = {0x2c, 0x29, 0x2b, 0x2a}; //The address of i2c
 //motorspeed : speed of motor on a scale from [0, 255] --> [stop, full speed]
 //res_mtr_out : confirmation of writing to motor
 int res_mtr_out, motorspeed_lowerbits;
 uint8_t motorspeed[2] = { 0 };
 unsigned char sensor_bytes2[24];
 int start_motor = 0x10; //16
    
    while(SYSTEM_RUN == true) {
  
    //duplicate action in imu.h, why not use that function????
        
	SImu_data imu_data;
	tcflush(usb_imu, TCIFLUSH);
	res1 = read(usb_imu,&sensor_bytes2[0],24);
        
    //function in imu.h that distributes imu_data to each field
    unpack_data(imu_data, sensor_bytes2);
 
	if(CONTROLLER_RUN == false)
    {   //if controller is not running
            //get initial value for psi from imu, set motorspeed to 0, check result of motorspeed assignment (~1/-1)
        psi_init  =  imu_data.psi;
        motorspeed[0] = 0 ;
        res_mtr_out = write(i2cHandle, &motorspeed,1);
    }
        
	imu_data.psi = imu_data.psi - psi_init;

	//printf("phi, theta, psi, phi_dot, theta_dot, psi_dot: %E, %E, %E, %E, %E, %E \n", imu_data.phi, imu_data.theta, imu_data.psi, imu_data.phi_dot, imu_data.theta_dot, imu_data.psi_dot);
	tcflush(usb_imu, TCIFLUSH);
        
    //calculate error between desired and measured
    Sstate error = state_error(imu_data, phi_d, theta_d);
        
    //calculate thrust and desired acceleration
    U = thrust(error,U_trim, gains);

    	 //printf("Moment: %E %E %E\n",M[0],M[1],M[2]);
   	 //Calculating the forces of each motor
    
    ff = forces(forces(U,Ct,d);

                //WORKED UP TO HERE
   	int i;

    // motor calibration - not needed
//   	double a[4]={3.0107,    3.0107,    3.0107,    3.0107};
//   	double b[4]={1.4400,    1.4400,    1.4400,    1.4400};
//   	double c[4]={0.0000,    0.0000,    0.0000,    0.0000};
	
	// controls motors
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

		if(motor_out[i] > 2047.0)   motor_out[i]=2047.0;
		else if(motor_out[i] < 0.0)	 motor_out[i]=0.0;
		}
        

//(uint8_t)(tmpVal >> 3) & 0xff
	for(i=0;i<4;i++)
	{
    //tell computer that we will use the file descriptor as and I2C comm line: assign address to fd
	 ioctl(i2cHandle,I2C_SLAVE,address[i]);
	 //motorspeed = (int)motor_out[i] / 8;
	 motorspeed[0] = (uint8_t)(motor_out[i] >> 3) & 0xff;
	// motorspeed_lowerbits = (((int)motor_out[i] % 8) & 0x07);
	motorspeed[1] = (((uint8_t)motor_out[i] % 8) & 0x07);

	 res_mtr_out = write(i2cHandle, &motorspeed, 1);

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

void start_motor(int motor_out[], char motor){
//this is for testing to see which motor is which
    //loop through motor_out and set its speed to 30 out of 255
    
    cout << "Motor " << motor << " is running..." << endl;

    for(int i = 0; i < 4; i++){
        if(i != motor){ motor_out[i] = 0;}
        else{ motor_out[i] = 30;}
    }

}

void controller_on_off(bool CONTROLLER_RUN){
    if(CONTROLLER_RUN == false){
        printf("Controller ON!!\n");
        CONTROLLER_RUN = true;
        }
    else if(CONTROLLER_RUN == true){
        printf("Controller OFF!!\n");
        CONTROLLER_RUN = false;
        }
}
void display_on_off(bool DISPLAY_RUN){
    if(DISPLAY_RUN == false){
        printf("DISPLAY ON!!\n");
        DISPLAY_RUN = true;
    }
    else if(DISPLAY_RUN == true){
        printf("DISPLAY OFF!!\n");
        DISPLAY_RUN = false;
    }
}
Sstate state_error(SImu_data, float phi_d, float theta_d){
    //calculate error in RADIANS
    //  xxx_d is xxx_desired.  imu outputs  degrees, we convert to radians with factor PI/180
    Sstate error;
    error.phi = (-imu_data.phi + phi_d) * PI/180;
    error.theta = (-imu_data.theta + theta_d) * PI/180;
    error.psi = (-imu_data.psi + psi_d) * PI/180;
    error.phi_dot = (-imu_data.phi_dot) * PI/180;
    error.theta_dot = (-imu_data.theta_dot) * PI/180;
    error.psi_dot = (-imu_data.psi_dot) * PI/180;
    return error;
}
int[] thrust(Sstate error, int U_trim[], Sgains gains ){
    //calculate thrust and acceleration
    //U[0] thrust, U[1:3]: roll acc, pitch acc, yaw acc
    int U[4];
    U[0] = 30 + U_trim[0];
    U[1] = gains.kp_phi * error.phi + gains.kd_phi * error.phi_dot + U_trim[1];
    U[2] = gains.kp_theta * error.theta + gains.kd_theta * error.theta_dot + U_trim[2];
    U[3] = gains.kp_psi * error.psi + gains.kd_psi * error.psi_dot + U_trim[3];
    return U;
}
int[] forces(int U[], double Ct, double d ){
    //calculate forces from thrusts and accelerations
    //U[0] thrust, U[1:3]: roll acc, pitch acc, yaw acc
    int U[4];
    //WE ARE MULTIPLYING AN INTEGER AND A FLOAT ==> GETS PROMOTED TO FLOAT, MAYBE MAKE ff A FLOAT ARRAY< THEN CAST TO INT??
    ff[0] = (U[0]/4-(U[3]/(4*Ct))+(U[2]/(2*d)));
   	ff[1] = (U[0]/4+(U[3]/(4*Ct))-(U[1]/(2*d)));
   	ff[2] = (U[0]/4-(U[3]/(4*Ct))-(U[2]/(2*d)));
   	ff[3] = (U[0]/4+(U[3]/(4*Ct))+(U[1]/(2*d)));
    return ff;

}
int main(void)
{
 //pthread_t - is an abstract datatype that is used as a handle to reference the thread
 pthread_t threads[3];
 //Special Attribute for starting thread (?)
 pthread_attr_t attr;
 //sched_param is a structure that maintains the scheduleing paramterts
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

