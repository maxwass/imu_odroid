#include "controller.h"

struct timeval t_init, t_now, t_now_v;
double t_v, del_t_v, t_prev_v = 0.0;

volatile bool SYSTEM_RUN = true;
volatile bool CONTROLLER_RUN = true;
volatile bool DISPLAY_RUN = false;

int i2cHandle, usb_imu, res1;

double Ct=0.013257116418667*10;
double d=0.169;
//signal frequency = 1/frequency = frequency in Hz
int onesecond = 1000000;
int frequency = 200;
int signal_frequency = onesecond/frequency;

int max_thrust = 460;
int delta_thrust = 30;
//The address of i2c
int address[4] = {0x2b, 0x2a, 0x2c, 0x29};

Angles desired_angles;

Gains gains;

Control_command U_trim;

//this is where the raw vicon data is being held, must access
  //it through the 
concur_data<State> state_from_vicon(VICON_MEM_LOC, "vicon");


//create our motor objects - accesible from all threads
motor motor_1(1, address[0]);
motor motor_2(2, address[1]);
motor motor_3(3, address[2]);
motor motor_4(4, address[3]);


//executes input from host computer on motors, controller gains, displays, and controller
void *command_input(void *thread_id){

    cout << "INSIDE COMMAND_INPUT" << endl;

    while(SYSTEM_RUN) {
      
   // cout << " ==>inside command_input while loop" << endl;

 
    //discard data written to the object referred to by the file descriptor ("usb_xbee), TCIOFLUSH - flushes both
        //data received but not read, and data written but not transmitted.
	// tcflush(usb_xbee,TCIOFLUSH);
       
 //    //read 2 bytes from file descriptor "usb_xbee" into buffer starting at "buf"
 //        // which in this case is the command from host computer
 //        read(usb_xbee,buf,2);

 //          if (buf[0] == 0xBD){printf("recieved: %c\n",buf[1]);}
 //          else{printf("Problem reading from XBee: Wrong start byte!\n");}

	// tcflush(usb_xbee,TCIOFLUSH);
        
        unsigned char command;
       // char command = '0';
	cout <<"    please give input for command_input: " << endl; 
	unsigned char command = get_terminal_input();
        
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
                
            case 'a':
            case 'A':
                controller_on_off(CONTROLLER_RUN);
                break;
                
            case 'b':
            case 'B':
                display_on_off(DISPLAY_RUN);
                break;
                
            case 'c':
            case 'C':
		            U_trim.thrust = U_trim.thrust + delta_thrust;
                if(U_trim.thrust > max_thrust) {
                  printf("Maximum Thrust Reached: Cannot Increase Thrust\n");
                  U_trim.thrust = mac_thrust;}
                else {printf("Increase Thrust\n");}
		            cout << U_trim.thrust << endl;
		            break;
                
            case 'd':
            case 'D':
                printf("Decrease Thrust!\n");
                U_trim.thrust = U_trim.thrust - delta_thrust;
                break;
                
            case 'e':
            case 'E':
                printf("Increase kp_phi and kp_theta\n");
                gains.kp_phi   = gains.kp_phi   + 0.1;
                gains.kp_theta = gains.kp_theta + 0.1;
                break;
                
            case 'f':
            case 'F':
                printf("Decrease kp_phi and kp_theta\n");
                gains.kp_phi   = gains.kp_phi   - 0.1;
                gains.kp_theta = gains.kp_theta - 0.1;
                break;
                
            case 'g':
            case 'G':
                printf("Increase kd_phi and kd_theta\n");
                gains.kd_phi   = gains.kd_phi   + 0.3;
                gains.kd_theta = gains.kd_theta + 0.3;
                break;
                
            case 'h':
            case 'H':
                printf("Decrease kd_phi and kd_theta\n");
                gains.kd_phi   = gains.kd_phi   - 0.3;
                gains.kd_theta = gains.kd_theta - 0.3;
                break;
                
            case 'i':
            case 'I':
                printf("Increase theta_d ('theta desired') \n");
                desired_angles.theta = desired_angles.theta + 1.0;
                break;
                
            case 'j':
            case 'J':
                printf("Decrease theta_d ('theta desired') \n");
                desired_angles.theta = desired_angles.theta - 1.0;
                break;
                
            case 'k':
            case 'K':
                printf("Increase phi_d ('phi desired') \n");
                desired_angles.phi = desired_angles.phi + 1.0;
                break;
                
            case 'l':
            case 'L':
                printf("Decrease phi_d ('phi desired') \n");
                desired_angles.phi = desired_angles.phi - 1.0;
                break;

            case 'm':
            case 'M':
              printf("Reset Desired_positions to START\n");
              desired_positions.x = START.x;
              desired_positions.y = START.y;
              desired_positions.z = START.z;
              break;

            case 'n':
            case 'N':
              printf("Increase X desired_positions\n");
              desired_positions.x = desired_positions.x + delta_position;
              break;

            case 'o':
            case 'O':
              printf("Decrease X desired_positions\n");
              desired_positions.x = desired_positions.x - delta_position;
              break;
            case 'p':
            case 'P':
              printf("Increase Y desired_positions\n");
              desired_positions.y = desired_positions.y + delta_position;
              break;

              // skip q/Q - this is quit
            case 'r':
            case 'R':
              printf("Decrease Y desired_positions\n");
              desired_positions.y = desired_positions.y - delta_position;
              break;

            case 's':
            case 'S':
              printf("Increase Z desired_positions\n");
              desired_positions.z = desired_positions.z + delta_position;
              break;

            case 't':
            case 'T':
              printf("Decrease Z desired_positions\n");
              desired_positions.z = desired_positions.z - delta_position;
              break;

            case ' ':
            case '\n':
              printf("Control Landing: Not Implemented\n");
              t_landing = t;
              CTRL_LANDING = true;
              break;

        }

    }
    pthread_exit(NULL);
}
void *control_stabilizer(void *thread_id){
 
  cout << "INSIDE CONTROL_STABALIZER" << endl;

 State imu_data; 

    while(SYSTEM_RUN) {

	      //flushed input buffer, reads input from imu (in degrees), distributes into fields of imu_data
	      get_data(usb_imu, imu_data);

        //calculate error (in radians) between desired and measured state
        State error = state_error(imu_data, desired_angles);
        
        //calculate thrust and desired acceleration
        Control_command U = thrust(error, U_trim, gains);

   	    //calculate the forces of each motor and change force on motor objects
         // and send via i2c
        set_forces(U,Ct,d);

	if(DISPLAY_RUN) { display_info(imu_data, error, U); }

    }
  cout << "EXIT CONTROL_STABILIZER" << endl;

    pthread_exit(NULL);
}
void *motor_signal(void *thread_id){

  cout << "INSIDE MOTOR_SIGNAL" << endl;

      while(SYSTEM_RUN){
  //cout << "SENDING FORCES" << endl;
	motor_1.send_force_i2c();
	motor_2.send_force_i2c();
	motor_3.send_force_i2c();
	motor_4.send_force_i2c();
	//send at about 200Hz
	usleep(200);
	}

  cout << "EXIT MOTOR_SIGNAL" << endl;

   pthread_exit(NULL);
}
void init(void){

    usleep(10000);

    printf("opening usb port for imu...\n");
    usb_imu = open_port();
     if (usb_imu > 0)
        printf("Done!\n");
     else
        printf("Fail to open usb port!\n");

    set_desired_angles(desired_angles);
    set_gains(gains);
    set_Utrim(U_trim);
}
void start_motors(void){
    //set speed to 30 out of 255
    cout << "Starting Motors ..." << endl;

    motor_1.set_force(30, CONTROLLER_RUN);
    motor_2.set_force(30, CONTROLLER_RUN);
    motor_3.set_force(30, CONTROLLER_RUN);
    motor_4.set_force(30, CONTROLLER_RUN);
}
void stop_motors(void){
    cout << "Stopping Motors ..." << endl;

    motor_1.shut_down();
    motor_2.shut_down();
    motor_3.shut_down();
    motor_4.shut_down();
}
void controller_on_off(bool &CONTROLLER_RUN){
    if(CONTROLLER_RUN == false){
        printf("Controller ON!!\n");
        CONTROLLER_RUN = true;
        }
    else{
        printf("Controller OFF!!\n");
        CONTROLLER_RUN = false;
        }
}
void display_on_off(bool& DISPLAY_RUN){
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
void set_desired_angles(Angles& desired_angles){
    desired_angles.theta = 0.0;
    desired_angles.phi   = 0.0;
    desired_angles.psi   = 0.0;
}
state state_error(const State& imu_data, const Angles& desired_angles){
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
    U.thrust = 30 + U_trim.thrust;
    U.roll_acc  = gains.kp_phi   * error.phi   + gains.kd_phi   * error.phi_dot   + U_trim.roll_acc;
    U.pitch_acc = gains.kp_theta * error.theta + gains.kd_theta * error.theta_dot + U_trim.pitch_acc;
    U.yaw_acc   = gains.kp_psi   * error.psi   + gains.kd_psi   * error.psi_dot   + U_trim.yaw_acc;
    return U;
}
void set_forces(const Control_command& U, double Ct, double d){
      //calculate forces from thrusts and accelerations
      double force_1 = (U.thrust/4 - (U.yaw_acc /(4*Ct)) + (U.pitch_acc /  (2*d)));
      double force_2 = (U.thrust/4 + (U.yaw_acc /(4*Ct)) - (U.roll_acc  /  (2*d)));
      double force_3 = (U.thrust/4 - (U.yaw_acc /(4*Ct)) - (U.pitch_acc /  (2*d)));
      double force_4 = (U.thrust/4 + (U.yaw_acc /(4*Ct)) + (U.roll_acc  /  (2*d)));

      //round forces to be integers
      motor_1.set_force( round(force_1), CONTROLLER_RUN );
      motor_2.set_force( round(force_2), CONTROLLER_RUN );
      motor_3.set_force( round(force_3), CONTROLLER_RUN );
      motor_4.set_force( round(force_4), CONTROLLER_RUN );
}
//intializes the curses: getch()
//initscr();
char get_terminal_input(void)
{//dissect and comment what is going on!
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return (char) ch;
}
void display_info(const State& imu_data, const State& error, const Control_command& U ){
        printf("<==========================================>\n");   	
	      printf("Controller ON \n");
       	printf("    IMU DATA    \n");
        printf("phi: %.2f         phi dot: %.2f\n", imu_data.phi, imu_data.phi_dot);
        printf("theta: %.2f         theta dot: %.2f\n",imu_data.theta, imu_data.theta_dot);
        printf("psi: %.2f         psi dot: %.2f\n\n\n",imu_data.psi, imu_data.psi_dot);

       // printf("    GAINS       \n");
       // printf("kp_phi: %f  kd_phi: %f\n",gains.kp_phi, gains.kd_phi);
       // printf("kp_theta: %f    kd_theta: %f\n",gains.kp_theta, gains.kd_theta);
       // printf("kp_psi: %f  kd_psi: %f\n\n\n",gains.kp_psi, gains.kd_psi);

        printf("    Errors (rad or degrees: currently degrees)      \n");
        printf("e_phi: %f,  e_theta: %f,    e_psi: %f\n\n\n",error.phi, error.theta, error.psi);

        printf("     Acceleration (N/s^2)      \n");
        printf("roll_acc: %f,   pitch_acc: %f,    yaw_acc: %f \n\n\n", U.roll_acc, U.pitch_acc, U.yaw_acc);

	      printf("    Thrust (0-255)     \n");
	      printf("thrust: %i \n\n\n", U.thrust);

        printf("    Forces (0-255)     \n");
        printf("motor_1: %i,  motor_2: %i,   motor_3: %i,    motor_4: %i \n\n\n", motor_1.get_force(), motor_2.get_force() , motor_3.get_force(), motor_4.get_force());
}
void configure_threads(void){
//pthread_t - is an abstract datatype that is used as a handle to reference the thread
    //threads[0] = control_stabilizer
    //threads[1] = motor_signal
    //threads[2] = command_input

    pthread_t threads[NUM_THREADS];
    //Special Attribute for starting thread
    pthread_attr_t attr;
    //sched_param is a structure that maintains the scheduling parameters
        //sched_param.sched_priority  - an integer value, the higher the value the higher a thread's proiority for scheduling
    struct sched_param param;
    int fifo_max_prio, fifo_min_prio;
    
     system("clear");

     cout << "INSIDE CONFIGURE_THREADS" << endl;
    
     gettimeofday(&t_init,NULL);
    
     // Set thread attributes: FIFO scheduling, Joinable
     // the sched_param.sched_priorirty is an int that must be in [min,max] for a certain schedule policy, in this case, SCHED_FIFO
     pthread_attr_init(&attr);
     pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
     fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
     fifo_min_prio = sched_get_priority_min(SCHED_FIFO); 

     	// Create threads
     
     cout << "=> creating control_stabilizer thread" << endl;
     // Higher priority for filter
     param.sched_priority = fifo_max_prio;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&threads[0], &attr, control_stabilizer, (void *) 0);

     cout << "=> creating motor_signal thread" << endl;
     // Medium priority for motor_signal
     param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&threads[1], &attr, motor_signal, (void *) 1);

     cout << "=> creating command_input thread" << endl;
     // Lower priority for vicon
     param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&threads[2], &attr, command_input, (void *) 2);
    

     // Wait for all threads to complete
       for (int i = 0; i < NUM_THREADS; i++)  {  
	   //calling join will block this main thread until every thread exits
         pthread_join(threads[i], NULL);
        }

     cout << "EXITING CONFIGURE_THREADS" << endl;
     
     close(usb_imu);
    
     pthread_attr_destroy(&attr);
}


int main(void){
	//intialize desired angles, gains, U_trim, & open port ot xbee and imu
	init();

	usleep(onesecond);
    
	start_motors();

	configure_threads();

	return 0;
}

