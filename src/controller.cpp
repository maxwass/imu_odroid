#include "controller.h"
// compiling: g++ controller.cpp -I ~/Desktop/quadrotor/imu_odroid/include/
//g++ controller.cpp vicon.cpp motor.cpp imu.cpp -I ~/Desktop/quadrotor/imu_odroid/include/ -lpthread -lcurses

//initialize process-scoped data-structures
Times times;
Positions init_positions;
Positions desired_positions;
Gains gains;
Control_command U_trim;

bool SYSTEM_RUN = true;
bool CONTROLLER_RUN = true;
bool DISPLAY_RUN = true;

int i2cHandle, usb_imu, usb_xbee, res1;

double Ct=0.013257116418667*10;
double d=0.169;
//signal frequency = 1/frequency = frequency in Hz
int onesecond = 1000000;
int frequency = 200;
int signal_frequency = onesecond/frequency;

float delta_position = 0.1;

int max_thrust = 460;
int delta_thrust = 30;
//The address of i2c
int address[4] = {0x2b, 0x2a, 0x2c, 0x29};

//create our motor objects - accesible from all threads
motor motor_1(1, address[0]);
motor motor_2(2, address[1]);
motor motor_3(3, address[2]);
motor motor_4(4, address[3]);

//this object is where the raw vicon data is being held, must access
  //it through this interface (2 methods: update and retrieve )
//concur_data<State> state_from_vicon(VICON_MEM_LOC, "vicon");
//shared_data<Vicon> vicon("v_mem_loc",VICON,"open");
//shared_data<State> imu("imu_mem_loc",IMU,"open");
//shared_data<Angles> des_angles("ang_mem_loc",DES_ANGLE. "create");


//executes input from host computer on motors, controller gains, displays, and controller
void *command_input(void *thread_id){
    cout << "INSIDE COMMAND_INPUT" << endl;
    unsigned char command;

    while(SYSTEM_RUN) {
	    cout <<"    please give input for command_input: ";// << endl; 
	    command = getchar(); 
        cout << endl;
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
                CONTROLLER_RUN = false;//change to System Run!
                break;
                
            case 'a':
            case 'A':
                controller_on_off(CONTROLLER_RUN);
                break;
                
            case 'b':
            case 'B':
                display_on_off(DISPLAY_RUN);
                system("clear");
                break;
                
            case 'c':
            case 'C':
                U_trim.thrust = U_trim.thrust + delta_thrust;
                if(U_trim.thrust > max_thrust) {
                  printf("Maximum Thrust Reached: Cannot Increase Thrust\n");
                  U_trim.thrust = max_thrust;}
                else {printf("Increase Thrust\n");}
		      cout << U_trim.thrust << endl;
		       break;
                
            case 'd':
            case 'D':
                printf("Decrease Thrust!\n");
                U_trim.thrust = U_trim.thrust - delta_thrust;
                cout << U_trim.thrust << endl;
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
              printf("Reset Desired_positions to initial values\n");
              desired_positions.x = init_positions.x;
              desired_positions.y = init_positions.y;
              desired_positions.z = init_positions.z;
              break;

            case 'j':
            case 'J':
	          printf("Increase X desired_positions\n");
              desired_positions.x = desired_positions.x - delta_position;
              break;
              break;

            case 'k':
            case 'K':
              printf("Decrease X desired_positions\n");
              desired_positions.x = desired_positions.x - delta_position;
              break;

            case 'l':
            case 'L':
              printf("Increase Y desired_positions\n");
              desired_positions.y = desired_positions.y + delta_position;
              break;

              // skip q/Q - this is quit
            case 'm':
            case 'M':
              printf("Decrease Y desired_positions\n");
              desired_positions.y = desired_positions.y - delta_position;
              break;

            case 'n':
            case 'N':
              printf("Increase Z desired_positions\n");
              desired_positions.z = desired_positions.z + delta_position;
              break;

            case 'o':
            case 'O':
              printf("Decrease Z desired_positions\n");
              desired_positions.z = desired_positions.z - delta_position;
              break;

            case ' ':
           // case '\n':
            //  printf("Control Landing: Not Implemented\n");
              //t_landing = t;
              //CTRL_LANDING = true;
              break;

        }

    }
    pthread_exit(NULL);
}
void *control_stabilizer(void *thread_id){
 
    cout << "INSIDE CONTROL_STABALIZER" << endl;

    State imu_data; 
    //weights is used for filter: current, one value ago, 2 values ago
    Weights weights = {.7,.2,.1};

    //for error calculations PID: stores the actual errors, not gains
    State_Error vicon_error;

    //position from raw data
    Vicon new_vicon,          old_vicon,          old_old_vicon          = {0};  
    Vicon new_filt_vicon,     old_filt_vicon,     old_old_filt_vicon     = {0};

    //velocity from raw data
    Vicon new_vicon_vel,      old_vicon_vel,      old_old_vicon_vel      = {0};
    Vicon new_filt_vicon_vel, old_filt_vicon_vel, old_old_filt_vicon_vel = {0};


//for display
    State imu_error = {0};
    Control_command U = {0};

    while(SYSTEM_RUN) {
    
        //flushed input buffer, reads input from imu (in degrees), distributes into fields of imu_data
        get_imu_data(usb_imu, imu_data);
            	//imu.retrieve(imu_data); //delete line above once implemented
        //calc new times and delta
        time_calc(times);    
       
        //get vicon data
	    get_vicon_data(usb_xbee, new_vicon);
        //filter vicon data
        new_filt_vicon = filter_vicon_data(new_vicon, old_vicon, old_old_vicon, weights);

        //calc velocities from vicon
        new_vicon_vel = vicon_velocity(new_filt_vicon, old_filt_vicon);
        //filter velocities
        new_filt_vicon_vel = filter_vicon_data(new_vicon_vel, old_vicon_vel, old_old_vicon_vel, weights);       
    
 //set old_old data to old_data, and old_data to new data
        //vicon data
        pushback(new_vicon,      old_vicon,      old_old_vicon);
        pushback(new_filt_vicon, old_filt_vicon, old_old_filt_vicon);

        //calculated vicon velocities
        pushback(new_vicon_vel,      old_vicon_vel,      old_old_vicon_vel);
        pushback(new_filt_vicon_vel, old_filt_vicon_vel, old_old_filt_vicon_vel);

        //saturate vicon velocities in calculations
        
        //calculate error from vicon
        error_vicon(vicon_error, new_filt_vicon, new_filt_vicon_vel, desired_positions,times);

    	//calculate desired attitude (phi theta phi)
    	Angles desired_angles = angles(vicon_error, gains);
    	
	     //calculate error from imu (in radians) between desired and measured state
        State imu_error = error_imu(imu_data, desired_angles);
    	
        //calculate thrust and desired acceleration
        Control_command U = thrust(imu_error,vicon_error, U_trim, gains);
    	
          //calculate the forces of each motor and change force on motor objects
          // and send via i2c 
        //set_forces(U,Ct,d);
	 
        if(DISPLAY_RUN) { display_info(imu_data, vicon_error, imu_error, U, new_vicon, new_filt_vicon, new_vicon_vel, new_filt_vicon_vel, desired_angles); }
  
    }
  
    cout << "EXIT CONTROL_STABILIZER" << endl;

    pthread_exit(NULL);
}
Angles angles(const State_Error& error, const Gains& gains){

Angles a;
       // phi_d =   kp_g*ey + ki_g*ey_i - kd_g*vel_filtered[1];
       // theta_d =  - kp_g*ex - ki_g*ex_i + kd_g*vel_filtered[0]

a.psi = 0;
a.phi     =  gains.kp_y*error.y.prop - gains.kd_y*error.y.deriv + gains.ki_y*error.y.integral;
a.theta   = -gains.kp_x*error.x.prop + gains.kd_x*error.x.deriv - gains.ki_x*error.x.integral;
return a;

}
State_Error error_vicon(State_Error& error, const Vicon& pos_filt, const Vicon& vel_filt, const Positions& desired_positions, const Times& times){
       
        //proportional errors:  desired_positions - filtered_positions
        error.x.prop = desired_positions.x - pos_filt.x;
        error.y.prop = desired_positions.y - pos_filt.y;
        error.z.prop = desired_positions.z - pos_filt.z;
       
        //derivative errors: desired_velocities - filtered_velocities
        error.x.deriv = 0 - vel_filt.x;
        error.y.deriv = 0 - vel_filt.y;
        error.z.deriv = 0 - vel_filt.z;
       
        //integral errors: integral error + (proportional error * delta_t)
        error.x.integral = error.x.integral + (error.x.prop * tv2float(times.delta));
        error.y.integral = error.y.integral + (error.y.prop * tv2float(times.delta));
        error.z.integral = error.z.integral + (error.z.prop * tv2float(times.delta));

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

    printf("opening usb port for imu...\n");
    usb_imu = open_imu_port();
     if (usb_imu > 0)
        printf("Done!\n");
     else
        printf("Fail to open usb port!\n");

    printf("opening usb port for vicon...\n");
    usb_xbee = open_vicon_port();
     if (usb_xbee > 0)
        printf("Done!\n");
     else
        printf("Fail to open vicon xbee port!\n");
    set_gains(gains);
    set_Utrim(U_trim);
    set_initial_times(times);

    // initialize shell 
  //  initscr();               /* Start curses mode        */
    struct termios oldattr, newattr;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr);
    
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

    gains.kp_x = 19.5;
    gains.kd_x = 2.7;
    gains.ki_x = .05;

    gains.kp_y = gains.kp_x;
    gains.kd_y = gains.kd_x;
    gains.ki_y = gains.ki_x;

    gains.kp_z = 12.0;
    gains.kd_z = 5.0;
    gains.ki_z = 5.0;
}
void set_initial_times(Times& times){
       clock_gettime(CLOCK_REALTIME,&(times.current));
       clock_gettime(CLOCK_REALTIME,&(times.old));
       clock_gettime(CLOCK_REALTIME,&(times.old_old));
       clock_gettime(CLOCK_REALTIME,&(times.delta));
       
}
void set_initial_positions(Positions& init_positions){
	init_positions.x = 0;
	init_positions.y = 0;
	init_positions.z = 0;
}
void set_timespec(timespec& x, timespec& y){
    //set x to equal y

      //truct timeval {
      //              time_t      tv_sec;     /* seconds */
      //               suseconds_t tv_usec;    /* microseconds */
      //           };
    
    x.tv_sec = y.tv_sec;
    x.tv_nsec = y.tv_nsec;
}
void time_calc(Times& times){
        //get current time, swap current and past, calc delta_t

//printf("current time: %.8f  old time: %.8f  delta_t: %.8f\n", tv2float(times.current), tv2float(times.old), tv2float(times.delta));

//update current time
clock_gettime(CLOCK_REALTIME,&(times.current));

//delta = x-y
//timeval_subtract(timeval Result, timeval x, timeval y)
//timeval_subtract(&(times.delta),&(times.current),&(times.old));
times.delta = diff(times.old, times.current);

//shift times back
//set time_old_old to time_old
set_timespec(times.old_old, times.old);

//set time_old to current time
set_timespec(times.old, times.current);
}

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {  
            temp.tv_sec = end.tv_sec-start.tv_sec;
            temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;

}
double tv2float (const timespec& time){
     return ((double) time.tv_sec + (time.tv_nsec / 1000000000.0)) ;
}
State error_imu(const State& imu_data, const Angles& desired_angles){
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
Control_command thrust(const State& imu_error, const State_Error& vicon_error, const Control_command& U_trim, const Gains& gains){
    //calculate thrust and acceleration
    //U[0] thrust, U[1:3]: roll acc, pitch acc, yaw acc
    Control_command U;
    int raw_thrust = (int) (-(gains.kp_z * vicon_error.z.prop)  -  (gains.kd_z * vicon_error.z.deriv) - (gains.ki_z * vicon_error.z.integral));

    U.thrust    =  raw_thrust + U_trim.thrust;
    U.roll_acc  =  (gains.kp_phi   * imu_error.phi  )  +  (gains.kd_phi   * imu_error.phi_dot  )  + U_trim.roll_acc;
    U.pitch_acc =  (gains.kp_theta * imu_error.theta)  +  (gains.kd_theta * imu_error.theta_dot)  + U_trim.pitch_acc;
    U.yaw_acc   =  (gains.kp_psi   * imu_error.psi  )  +  (gains.kd_psi   * imu_error.psi_dot  )  + U_trim.yaw_acc;
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
Vicon vicon_velocity(Vicon& current, Vicon& old){
    
    Vicon velocity = {0.0};
    
    velocity.x     = (current.x - old.x)/tv2float(times.delta);
    velocity.y     = (current.y - old.y)/tv2float(times.delta);
    velocity.z     = (current.z - old.z)/tv2float(times.delta);
    velocity.theta = (current.theta - old.theta)/tv2float(times.delta);
    velocity.phi   = (current.phi - old.phi)/tv2float(times.delta);
    velocity.psi   = (current.psi - old.psi)/tv2float(times.delta);

    return velocity;    
}
void display_info(const State& imu_data, const State_Error& vicon_error, const State& imu_error, const Control_command& U, const Vicon& vicon, const Vicon& vicon_filt, const Vicon& vicon_vel, const Vicon& vicon_vel_filt, const Angles& desired_angles){
    system("clear");
        cout << "IN DISPLAY_INFO" << endl;
    printf("<==========================================>\n");   	
    printf("Controller ON \n");
        printf("    IMU DATA (degrees)    \n");
        printf("phi: %.2f         phi dot: %.2f\n", imu_data.phi, imu_data.phi_dot);
        printf("theta: %.2f         theta dot: %.2f\n",imu_data.theta, imu_data.theta_dot);
        printf("psi: %.2f         psi dot: %.2f\n\n\n",imu_data.psi, imu_data.psi_dot);

	printf("    vicon data    \n");
        printf("phi: %.2f         x: %.2f\n", vicon.phi, vicon.x);
        printf("theta: %.2f       y: %.2f\n",vicon.theta, vicon.y);
        printf("psi: %.2f         z: %.2f\n\n\n",vicon.psi, vicon.z);

    
 	printf("   filtered vicon data    \n");
        printf("phi: %.2f         x: %.2f\n", vicon_filt.phi, vicon_filt.x);
        printf("theta: %.2f       y: %.2f\n",vicon_filt.theta, vicon_filt.y);
        printf("psi: %.2f         z: %.2f\n\n\n",vicon_filt.psi, vicon_filt.z);
 
    printf("    vicon velocity    \n");
        printf("phi_dot: %.2f         x_dot: %.2f\n", vicon_vel.phi, vicon_vel.x);
        printf("theta_dot: %.2f       y_dot: %.2f\n",vicon_vel.theta, vicon_vel.y);
        printf("psi_dot: %.2f         z_dot: %.2f\n\n\n",vicon_vel.psi, vicon_vel.z);

    printf("   filtered vicon velocity    \n");
        printf("phi_dot: %.2f         x_dot: %.2f\n", vicon_vel_filt.phi, vicon_vel_filt.x);
        printf("theta_dot: %.2f       y_dot: %.2f\n",vicon_vel_filt.theta, vicon_vel_filt.y);
        printf("psi_dot: %.2f         z_dot: %.2f\n\n\n",vicon_vel_filt.psi, vicon_vel_filt.z);

    printf("     DESIRED ANGLES = f(vicon_error, gains)      \n");
        printf("phi: %.2f\n", desired_angles.phi);
        printf("theta: %.2f\n", desired_angles.theta);
        printf("psi: %.2f\n\n\n", desired_angles.psi);

/*
    printf("    GAINS       \n");
        printf("kp_phi: %f  kd_phi: %f\n",gains.kp_phi, gains.kd_phi);
        printf("kp_theta: %f    kd_theta: %f\n",gains.kp_theta, gains.kd_theta);
        printf("kp_psi: %f  kd_psi: %f\n\n\n",gains.kp_psi, gains.kd_psi);
*/

    printf("  IMU ERRORS = f(imu_data, desired_angles) (radians)      \n");
        printf("e_phi: %.2f\n", imu_error.phi);
        printf("e_theta: %.2f\n", imu_error.theta);
        printf("e_psi: %.2f\n\n\n", imu_error.psi);

    printf("  VICON ERRORS (meters)      \n");
        printf("x_prop: %.2f      y_prop %.2f       z_prop %.2f\n", vicon_error.x.prop, vicon_error.y.prop, vicon_error.z.prop);
        printf("x_deriv: %.2f      y_deriv %.2f       z_deriv %.2f\n", vicon_error.x.deriv, vicon_error.y.deriv, vicon_error.z.deriv);
        printf("x_integ: %.2f      y_integ %.2f       z_integ %.2f\n\n\n", vicon_error.x.integral, vicon_error.y.integral, vicon_error.z.integral);
/*
    printf("     ACCELERATION (N/s^2)      \n");
        printf("roll_acc: %.2f\n", U.roll_acc);
        printf("pitch_acc: %.2f\n", U.pitch_acc);
        printf("yaw_acc: %.2f\n\n\n", U.yaw_acc);
*/
	printf("    THRUST (0-255)     \n");
	    printf("thrust: %f \n\n\n", U.thrust);

    printf("    FORCES (0-255)     \n");
        printf("motor_1: %.2i\n", motor_1.get_force());
        printf("motor_2: %.2i\n", motor_2.get_force());
        printf("motor_3: %.2i\n", motor_3.get_force());
        printf("motor_4: %.2i\n", motor_4.get_force());

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
    
    // system("clear");

     cout << "INSIDE CONFIGURE_THREADS" << endl;
    
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
     // Lower priority for commmand input
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

