#include "motor_test.h"

pthread_mutex_t data_acq_mutex; 
pthread_cond_t data_acq_cv;

struct timeval t_init, t_now, t_now_v;
double t_v, del_t_v, t_prev_v = 0.0;

volatile bool SYSTEM_RUN = true;
volatile bool CONTROLLER_RUN = false;
volatile bool DISPLAY_RUN = true;

int i2cHandle, usb_imu, usb_xbee, res1;

double Ct=0.013257116418667*10;
double d=0.169;

//The address of i2c
int address[4] = {0x2c, 0x29, 0x2b, 0x2a};

//finding motor addresses:
    //different pad combinations make 4 different addresses
    //download i2c library - i2c-tools : sudo apt_get ... i2c-tools
    //check for i2c addresses: sudo i2cdetect -y -l
            // prints out i2c ports on odroid - need find which pin corresponds to which output i2c
             

// IT SEEMS YOU CANNOT CALL FUNCTIONS IN THE PREPROCESSOR AREA???
Desired_angles desired_angles;

Gains gains;

Control_command U_trim;

//create our motor objects
motor motor_1(1, address[0]);
motor motor_2(2, address[1]);
motor motor_3(3, address[2]);
motor motor_4(4, address[3]);


//executes input from host computer on motors, controller gains, displays, and controller
void *command_input(void *thread_id){
 //command from the host computer
 unsigned char buf[2];
    
    while(SYSTEM_RUN == true) {
        
    //discard data written to the object referred to by the file descriptor ("usb_xbee), TCIOFLUSH - flushes both
        //data received but not read, and data written but not transmitted.
	tcflush(usb_xbee,TCIOFLUSH);
    //   
    //read 2 bytes from file descriptor "usb_xbee" into buffer starting at "buf" which in this case is the command
        //from host computer
//	read(usb_xbee,buf,2);

 //       if (buf[0] == 0xBD){printf("recieved: %c\n",buf[1]);}
 //       else{printf("Wrong start byte!\n");}
    
    //Why are we flushing HERE???
//	tcflush(usb_xbee,TCIOFLUSH);

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
                start_motors();
                break;
                
            case '5':
            case 'q':
            case 'Q':
                printf("Motor Stop!\n");
                stop_motors();
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
                U_trim.thrust = U_trim.thrust + 4;
                break;
                
            case 'f':
            case 'F':
                printf("Decrease Thrust!\n");
                U_trim.thrust = U_trim.thrust - 4;
                break;
                
            case 'w':
            case 'W':
                printf("Increase kp_phi and kp_theta\n");
                gains.kp_phi   = gains.kp_phi   + 0.1;
                gains.kp_theta = gains.kp_theta + 0.1;
                break;
                
            case 's':
            case 'S':
                printf("Decrease kp_phi and kp_theta\n");
                gains.kp_phi   = gains.kp_phi   - 0.1;
                gains.kp_theta = gains.kp_theta - 0.1;
                break;
                
            case 'e':
            case 'E':
                printf("Increase kd_phi and kd_theta\n");
                gains.kd_phi   = gains.kd_phi   + 0.3;
                gains.kd_theta = gains.kd_theta + 0.3;
                break;
                
            case 'd':
            case 'D':
                printf("Decrease kd_phi and kd_theta\n");
                gains.kd_phi   = gains.kd_phi   - 0.3;
                gains.kd_theta = gains.kd_theta - 0.3;
                break;
                
            case 'k':
            case 'K':
                printf("Increase theta_d ('theta desired') \n");
                desired_angles.theta = desired_angles.theta + 1.0;
                break;
                
            case 'i':
            case 'I':
                printf("Decrease theta_d ('theta desired') \n");
                desired_angles.theta = desired_angles.theta - 1.0;
                break;
                
            case 'l':
            case 'L':
                printf("Increase phi_d ('phi desired') \n");
                desired_angles.phi = desired_angles.phi + 1.0;
                break;
                
            case 'j':
            case 'J':
                printf("Decrease phi_d ('phi desired') \n");
                desired_angles.phi = desired_angles.phi - 1.0;
                break;
                
         //   default:
         //       printf("Command Input: Unrecognized input command");
         //       cout << command << endl;
         //       break;
        }

    }
    pthread_exit(NULL);
}
void *control_stabilizer(void *thread_id){
 
 unsigned char sensor_bytes2[24];
 int start_motor = 0x10; //16
    
 State imu_data; 

    while(SYSTEM_RUN) {

        if(CONTROLLER_RUN == false) {   stop_motors();  }

	    tcflush(usb_imu, TCIFLUSH);
printf("1\n");

	 //   res1 = read(usb_imu,&sensor_bytes2[0],24);
	 
printf("2\n");        
        //distributes data from imu stored in buffer sensor_bytes2 to 
            //each field in imu_data
       // unpack_data(imu_data, sensor_bytes2);
         imu_data.psi = 0.;
    imu_data.theta = 0.;
    imu_data.phi = 0.;
    imu_data.phi_dot = 0.;
    imu_data.theta_dot = 0.;
    imu_data.psi_dot =  0.;

	    tcflush(usb_imu, TCIFLUSH);
        
        //calculate error between desired and measured state
        State error = state_error(imu_data, desired_angles);
        
        //calculate thrust and desired acceleration
        Control_command U = thrust(error,U_trim, gains);

   	    //calculate the forces of each motor and change force on motor objects
        set_forces(U,Ct,d);

        //send forces to motor via I2C
        send_forces();

	    if(DISPLAY_RUN == true) { display_info(imu_data, error); }

    }

    pthread_exit(NULL);
}
void *buffer_thread(void *thread_id){
    cout << "called buffer_thread: no content yet" << endl;
    // while(SYSTEM_RUN == true) {

    // }
    pthread_exit(NULL);
}
void init(void){
    set_desired_angles(desired_angles);
    set_gains(gains);
    set_Utrim(U_trim);
}
void start_motors(void){
//this is for testing to see which motor is which
    //set speed to 30 out of 255
    
    cout << "Starting Motors ..." << endl;

    motor_1.set_force(30.0, CONTROLLER_RUN);
    motor_2.set_force(30.0, CONTROLLER_RUN);
    motor_3.set_force(30.0, CONTROLLER_RUN);
    motor_4.set_force(30.0, CONTROLLER_RUN);
}
void stop_motors(void){
 
    cout << "Stopping Motors ..." << endl;

    motor_1.shut_down();
    motor_2.shut_down();
    motor_3.shut_down();
    motor_4.shut_down();
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
void set_Utrim(Control_command& U_trim){
    U_trim.thrust    = 0.0;
    U_trim.roll_acc  = 0.0;
    U_trim.pitch_acc = 0.0;
    U_trim.yaw_acc   = 0.0;
}
void set_gains(Gains& gains){
    gains.kp_theta = 5.5;
    gains.kd_theta = 0.32;
    
    gains.kp_phi = 5.5;
    gains.kd_phi = 0.32;
    
    gains.kp_psi = 5.2;
    gains.kd_psi = 0.3;  
}
void set_desired_angles(Desired_angles& desired_angles){
    desired_angles.theta = 0.0;
    desired_angles.phi   = 0.0;
    desired_angles.psi   = 0.0;
}
state state_error(const State& imu_data, const Desired_angles& desired_angles){
    //calculate error in RADIANS
    //  xxx_d is xxx_desired.  imu outputs  degrees, we convert to radians with factor PI/180
    State error;
    error.phi       =     (-imu_data.phi    +   desired_angles.phi) * PI/180;
    error.theta     =     (-imu_data.theta  + desired_angles.theta) * PI/180;
    error.psi       =     (-imu_data.psi    +   desired_angles.psi) * PI/180;
    error.phi_dot   =                           (-imu_data.phi_dot) * PI/180;
    error.theta_dot =                         (-imu_data.theta_dot) * PI/180;
    error.psi_dot   =                           (-imu_data.psi_dot) * PI/180;
    return error;
}
Control_command thrust(const State& error, const Control_command& U_trim, const Gains& gains){
    //calculate thrust and acceleration
    //U[0] thrust, U[1:3]: roll acc, pitch acc, yaw acc
    Control_command U;
    U.thrust = 30 + U_trim.roll_acc;
    U.roll_acc  = gains.kp_phi   * error.phi   + gains.kd_phi   * error.phi_dot   + U_trim.roll_acc;
    U.pitch_acc = gains.kp_theta * error.theta + gains.kd_theta * error.theta_dot + U_trim.pitch_acc;
    U.yaw_acc   = gains.kp_psi   * error.psi   + gains.kd_psi   * error.psi_dot   + U_trim.yaw_acc;
    return U;
}
void set_forces(const Control_command& U, double Ct, double d){
    //calculate forces from thrusts and accelerations
    
      double force_1 = (U.thrust/4 - (U.yaw_acc  /(4*Ct))+(U.pitch_acc / (2*d)));
      //     ff[0]   = (U[0]    /4 - (U[3]      /(4*Ct))+(U[2]        / (2*d)));
      double force_2 = (U.thrust/4 - (U.yaw_acc /(4*Ct))+(U.roll_acc  /  (2*d)));
      //     ff[1]   = (U[0]    /4 + (U[3]      /(4*Ct))-(U[1]        / (2*d)));
      double force_3 = (U.thrust/4 - (U.yaw_acc /(4*Ct))-(U.pitch_acc /  (2*d)));
      //     ff[2]   = (U[0]    /4 - (U[3]      /(4*Ct))-(U[2]        / (2*d)));
      double force_4 = (U.thrust/4 - (U.yaw_acc /(4*Ct))+(U.roll_acc  /  (2*d)));
      //     ff[3]   = (U[0]    /4 + (U[3]      /(4*Ct))+(U[1]        / (2*d)));


      motor_1.set_force( round(force_1), CONTROLLER_RUN );
      motor_2.set_force( round(force_2), CONTROLLER_RUN );
      motor_3.set_force( round(force_3), CONTROLLER_RUN );
      motor_4.set_force( round(force_4), CONTROLLER_RUN );
}
void send_forces(void){
    //send forces to all motors via I2C
    motor_1.send_force_i2c();
    motor_2.send_force_i2c();
    motor_3.send_force_i2c();
    motor_4.send_force_i2c();
}
void display_info(const State& imu_data, const State& error){
        printf("<==========================================>\n");
        if(CONTROLLER_RUN == true) printf("Controller ON \n");
        else if (CONTROLLER_RUN == false) printf("Controller OFF \n");
        printf("    IMU DATA    \n");
        printf("phi: %.2f         phi dot: %.2f\n", imu_data.phi, imu_data.phi_dot);
        printf("theta: %.2f         theta dot: %.2f\n",imu_data.theta, imu_data.theta_dot);
        printf("psi: %.2f         psi dot: %.2f\n\n\n",imu_data.psi, imu_data.psi_dot);

        printf("    GAINS       \n");
        printf("kp_phi: %f  kd_phi: %f\n",gains.kp_phi, gains.kd_phi);
        printf("kp_theta: %f    kd_theta: %f\n",gains.kp_theta, gains.kd_theta);
        printf("kp_psi: %f  kd_psi: %f\n\n\n",gains.kp_psi, gains.kd_psi);

        printf("    MOTOR_OUT   \n");
        printf("Motor #1: %i\n",     motor_1.get_force());
        printf("Motor #2: %i\n",     motor_2.get_force());        
        printf("Motor #3: %i\n",     motor_3.get_force());
        printf("Motor #4: %i\n\n\n", motor_4.get_force());
        
        printf("    Errors      \n");
        printf("e_phi: %f,  e_theta: %f,    e_psi: %f\n\n\n",error.phi, error.theta, error.psi);
        //printf("Motor out: %d\n", motor_out[0]);
}


int main(void){
    cout <<"in main" << endl;
    init();
  //pthread_t - is an abstract datatype that is used as a handle to reference the thread
  pthread_t threads[3];
  //Special Attribute for starting thread (?)
  pthread_attr_t attr;
  //sched_param is a structure that maintains the scheduleing paramterts
  struct sched_param	param;
  int fifo_max_prio, fifo_min_prio;

  system("clear");
  gettimeofday(&t_init,NULL);
 
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
    
  //   close(motor::get_i2c());
     close(usb_xbee);
     close(usb_imu);
    
     pthread_attr_destroy(&attr);
     pthread_mutex_destroy(&data_acq_mutex);
     return 0;
}

