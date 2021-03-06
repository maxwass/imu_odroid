#include "controller.h"

//g++ controller.cpp vicon.cpp motor.cpp imu.cpp logger.cpp -I ~/Desktop/quadrotor/imu_odroid/include/ -lpthread -lncurses -lboost_system

//initialize process-scoped data-structures
std::string log_filename = "file.log";
logger logger(log_filename, 100);

Times times;
Times time_m;

Positions init_positions = {0.0};
Positions desired_positions = {0.0};
Gains gains;
Control_command U_trim = {0.0};

bool SYSTEM_RUN = true;
bool CONTROLLER_RUN = true;
bool DISPLAY_RUN = false;
bool LOG_DATA = true;

int i2cHandle, usb_imu, usb_xbee;

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


void *control_stabilizer(void *thread_id){
 
    printf("INSIDE CONTROL_STABALIZER\n");

    State imu_data; 
    //weights is used for filter: current, one value ago, 2 values ago
    Weights weights = {.7,.2,.1};

    //for error calculations PID: stores the actual errors, not gains
    State_Error vicon_error = {0.0};
    //vicon_error.zdd.integral = 0;
    
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
        //get_vicon_data(usb_xbee, new_vicon);
        
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
        set_forces(U,Ct,d);

        if (LOG_DATA)    { log_data(times, new_vicon, new_vicon_vel, new_filt_vicon, new_filt_vicon_vel, vicon_error, imu_data, imu_error, desired_angles);        }
        if (DISPLAY_RUN) { display_info(imu_data, vicon_error, imu_error, U, new_vicon, new_filt_vicon, new_vicon_vel, new_filt_vicon_vel, desired_angles, times, time_m); }
  
    }
  
    printf("EXIT CONTROL_STABILIZER\n");
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

   printf("INSIDE MOTOR_SIGNAL\n");
   set_initial_times(time_m);

      while(SYSTEM_RUN){
	motor_1.send_force_i2c();
	motor_2.send_force_i2c();
	motor_3.send_force_i2c();
	motor_4.send_force_i2c();
    
    time_calc(time_m);
	//send at about 500Hz
	usleep(200);
	}

   printf("EXIT MOTOR_SIGNAL\n");

   pthread_exit(NULL);
}
void init(void){
    
    //ncurses
    initscr();

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
     set_initial_times(times);
}
void start_motors(void){
    //set speed to 30 out of 255
    printf("Starting Motors ...\n");
    motor_1.set_force(30, CONTROLLER_RUN);
    motor_2.set_force(30, CONTROLLER_RUN);
    motor_3.set_force(30, CONTROLLER_RUN);
    motor_4.set_force(30, CONTROLLER_RUN);
}
void stop_motors(void){
    printf("Stopping Motors ...\n");
    motor_1.set_force(0, false);
    motor_2.set_force(0, false);
    motor_3.set_force(0, false);
    motor_4.set_force(0, false);
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
    int calc_thrust = (int) (-(gains.kp_z * vicon_error.z.prop)  -  (gains.kd_z * vicon_error.z.deriv) - (gains.ki_z * vicon_error.z.integral));

    U.thrust    =  calc_thrust + U_trim.thrust;
    U.roll_acc  =  (gains.kp_phi   * imu_error.phi  )  +  (gains.kd_phi   * imu_error.phi_dot  )  + U_trim.roll_acc;
    U.pitch_acc =  (gains.kp_theta * imu_error.theta)  +  (gains.kd_theta * imu_error.theta_dot)  + U_trim.pitch_acc;
    U.yaw_acc   =  (gains.kp_psi   * imu_error.psi  )  +  (gains.kd_psi   * imu_error.psi_dot  )  + U_trim.yaw_acc;
   
    if (U.thrust <= 0) U.thrust = 0;
   
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
void log_data(const Times& times, const Vicon& new_vicon, const Vicon& new_vicon_vel, const Vicon& new_filt_vicon, const Vicon& new_filt_vicon_vel, const State_Error& vicon_error, const State& imu_data, const State& imu_error, const Angles& desired_angles){

    Data_log d;
    d.time   = times;
    d.vicon_data      = new_vicon;
    d.vicon_vel       = new_vicon_vel;
    d.vicon_data_filt = new_filt_vicon;
    d.vicon_vel_filt  = new_filt_vicon_vel;
    d.vicon_error     = vicon_error;
    d.imu             = imu_data;
    d.imu_error       = imu_error;
    d.forces.motor_1  = *(motor_1.get_force());
    d.forces.motor_2  = *(motor_2.get_force());
    d.forces.motor_3  = *(motor_3.get_force());
    d.forces.motor_4  = *(motor_4.get_force());
    d.desired_angles  = desired_angles;
    
    logger.log(d);
}
void display_info(const State& imu_data, const State_Error& vicon_error, const State& imu_error, const Control_command& U, const Vicon& vicon, const Vicon& vicon_filt, const Vicon& vicon_vel, const Vicon& vicon_vel_filt, const Angles& desired_angles, const Times& times, const Times& time_m){
   // system("clear");
    clear();//function in curses library  
    printf("<==========================================>\n");   	
        printf("        IMU DATA (degrees)    \n");
        printf("phi:   %7.2f         phi dot:   %7.2f \n",imu_data.phi, imu_data.phi_dot, imu_data.phi, imu_data.phi_dot);
        printf("theta: %7.2f         theta dot: %7.2f\n",imu_data.theta, imu_data.theta_dot);
        printf("psi:   %7.2f         psi dot:   %7.2f\n\n",imu_data.psi, imu_data.psi_dot);

	printf("        VICON DATA                                                          FILTERED VICON DATA   \n");
        printf("phi:      %10.2f       x: %10.2f                           phi:   %10.2f       x: %10.2f\n", vicon.phi, vicon.x, vicon_filt.phi, vicon_filt.x);
        printf("theta:    %10.2f       y: %10.2f                           theta: %10.2f       y: %10.2f\n",vicon.theta, vicon.y, vicon_filt.theta, vicon_filt.y);
        printf("psi:      %10.2f       z: %10.2f                           psi:   %10.2f       z: %10.2f\n\n",vicon.psi, vicon.z, vicon_filt.psi, vicon_filt.z);
      
    printf("        VICON VELOCITY                                                      FILTERED VICON VELOCITY  \n");
printf("phi_dot:  %10.2f       x_dot: %10.2f                  phi_dot:   %10.2f      x_dot: %10.2f\n", vicon_vel.phi, vicon_vel.x,  vicon_vel_filt.phi, vicon_vel_filt.x);
printf("theta_dot:%10.2f       y_dot: %10.2f                  theta_dot: %10.2f      y_dot: %10.2f\n",vicon_vel.theta, vicon_vel.y, vicon_vel_filt.theta, vicon_vel_filt.y);
printf("psi_dot:  %10.2f       z_dot: %10.2f                  psi_dot:   %10.2f      z_dot: %10.2f\n\n",vicon_vel.psi, vicon_vel.z ,vicon_vel_filt.psi, vicon_vel_filt.z);

    printf("        VICON ERRORS (meters)      \n");
        printf("x_prop:   %10.2f      y_prop:  %10.2f       z_prop:  %10.2f\n", vicon_error.x.prop, vicon_error.y.prop, vicon_error.z.prop);
        printf("x_deriv:  %10.2f      y_deriv: %10.2f       z_deriv: %10.2f\n", vicon_error.x.deriv, vicon_error.y.deriv, vicon_error.z.deriv);
        printf("x_integ:  %10.2f      y_integ: %10.2f       z_integ: %10.2f\n\n", vicon_error.x.integral, vicon_error.y.integral, vicon_error.z.integral);

    printf("        DESIRED ANGLES = f(vicon_error, gains)      \n");
        printf("phi:      %10.2f\n", desired_angles.phi);
        printf("theta:    %10.2f\n", desired_angles.theta);
        printf("psi:      %10.2f\n\n", desired_angles.psi);

    printf("        IMU ERRORS = f(imu_data, desired_angles) (radians)      \n");
        printf("e_phi:    %10.2f\n", imu_error.phi);
        printf("e_theta:  %10.2f\n", imu_error.theta);
        printf("e_psi:    %10.2f\n\n", imu_error.psi);
    
    printf("        ACCELERATION (N/s^2)      \n");
        printf("roll_acc:     %7.2f\n", U.roll_acc);
        printf("pitch_acc:    %7.2f\n", U.pitch_acc);
        printf("yaw_acc:      %7.2f\n\n", U.yaw_acc);

	printf("        THRUST = Calc_Thrust + Trim_Thrust (0-255)     \n");
	    printf("thrust: %f \n\n", U.thrust);

    printf("        FORCES (0-255)     \n");
        printf("motor_1: %i  ", *(motor_1.get_force()));
        printf("motor_2: %i  ", *(motor_2.get_force()));
        printf("motor_3: %i  ", *(motor_3.get_force()));
        printf("motor_4: %i \n\n", *(motor_4.get_force()));

    printf("        TIME INFO     \n");
        printf("Controller timestep (s)       :     %7.4f\n", tv2float(times.delta));
        printf("Controller loop frequency (Hz):     %5.3f\n", 1/tv2float(times.delta));
        printf("Motor loop frequency (Hz)     :    %5.3f\n", 1/tv2float(time_m.delta));
       // printf("%lld.%.9ld", (long long)times.current.tv_sec, times.current.tv_nsec);
        printf("Time Date                     :    %s\n ", times.date_time);

        refresh();//refreshes shell console to output this text
}

//executes input from host computer on motors, controller gains, displays, and controller
void *command_input(void *thread_id){
   
    printf("INSIDE COMMAND_INPUT");
    unsigned char command;
    string input;

    while(SYSTEM_RUN) {
	    printf("    please give input for command_input: ");
        //command = getchar(); 
        //getline(cin, input);
        command = getch();
        printf("input: %i \n", command);
        switch (command) {
            case '1':

            case '4':
                start_motors();
                break;
                
            case '5':
            case 'q':
            case 'Q':
                printf("Motor Stop!\n");
                stop_motors();
                CONTROLLER_RUN = false;//change to System Run!
                endwin();
                break;
                
            case ' ':
            case 'z':
            case 'Z':
            case '\n':
                controller_on_off(CONTROLLER_RUN);
                break;

// control of desired positions
            case 'w':
            case 'W':
	          printf("Increase X desired_positions\n");
              desired_positions.x = desired_positions.x + delta_position;
                break; 
    
            case 's':
            case 'S':
	          printf("Decrease X desired_positions\n");
              desired_positions.x = desired_positions.x - delta_position;
                break; 
            
            case 'd':
            case 'D':
              printf("Increase Y desired_positions\n");
              desired_positions.y = desired_positions.y + delta_position;
              break;

            case 'a':
            case 'A':
              printf("Decrease Y desired_positions\n");
              desired_positions.y = desired_positions.y - delta_position;
                break;

            case 'r':
            case 'R':
              printf("Decrease Z desired_positions\n");
              desired_positions.z = desired_positions.z - delta_position;
                break;

            case 'f':
            case 'F':
              printf("Increase Z desired_positions\n");
              desired_positions.z = desired_positions.z + delta_position;
                 break;

// control of desired positions

            case 'b':
            case 'B':
                display_on_off(DISPLAY_RUN);
               // system("clear");
               //clear(); //function in curses library
                break;
                
            case 'c':
            case 'C':
                if((U_trim.thrust + delta_thrust) >=  max_thrust) {
                  printf("Maximum Thrust Reached: Cannot Increase Thrust\n");
                  U_trim.thrust = max_thrust;}
                else {U_trim.thrust += delta_thrust; 
                     printf("Increase Thrust: %f\n", U_trim.thrust);}
              break;
                
            case 'v':
            case 'V':
                if((U_trim.thrust-=delta_thrust) <= 0) {
                printf("Thrust is 0. Cannot decrease further\n");
                U_trim.thrust = 0;}
                else {U_trim.thrust -= delta_thrust;
                      printf("Decrease Thrust: %f\n", U_trim.thrust);}
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
                printf("Reset desired_positions to current position (raw vicon)\n");
                Vicon current_position;
                get_vicon_data(usb_xbee, current_position);
                desired_positions.x = current_position.x;
                desired_positions.y = current_position.y;
                desired_positions.z = current_position.z;
                break;

            case 'k':
            case 'K':
                printf("Increase kp_phi and kp_theta\n");
                gains.kp_phi   = gains.kp_phi   + 0.1;
                gains.kp_theta = gains.kp_theta + 0.1;
                break;
            
            case 'm':
            case 'M':
                printf("ecrease kp_phi and kp_theta\n");
                gains.kp_phi   = gains.kp_phi   - 0.1;
                gains.kp_theta = gains.kp_theta - 0.1;    
                break;

            case 'l':
            case 'L':
                printf("Start/Stop Logging\n");
                LOG_DATA = ~LOG_DATA;
                break;
                
              //used after j: s,w,r,f,v
              
              //  printf("Control Landing: Not Implemented\n");
              //t_landing = t;
              //CTRL_LANDING = true;

        }

    }
    pthread_exit(NULL);
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
     printf("INSIDE CONFIGURE_THREADS\n");
     
     // Set thread attributes: FIFO scheduling, Joinable
     // the sched_param.sched_priorirty is an int that must be in [min,max] for a certain schedule policy, in this case, SCHED_FIFO
     pthread_attr_init(&attr);
     pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
     fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
     fifo_min_prio = sched_get_priority_min(SCHED_FIFO); 

     	// reate threads
     
     printf("=> creating control_stabilizer thread\n");
     // Higher priority for filter
     param.sched_priority = fifo_max_prio;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&threads[0], &attr, control_stabilizer, (void *) 0);

     printf("=> creating motor_signal thread\n");
     // Medium priority for motor_signal
     param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&threads[1], &attr, motor_signal, (void *) 1);

     printf("=> creating command_input thread\n");
     // Lower priority for commmand input
     param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&threads[2], &attr, command_input, (void *) 2);
    

     // Wait for all threads to complete
       for (int i = 0; i < NUM_THREADS; i++)  {  
	   //calling join will block this main thread until every thread exits
         pthread_join(threads[i], NULL);
        }

     printf("EXITING CONFIGURE_THREADS\n");
     close(usb_imu);
    
     pthread_attr_destroy(&attr);
}


int main(void){
	//intialize desired angles, gains, U_trim, & open port ot xbee and imu

    init();
    
	usleep(onesecond);

    configure_threads();
    return 0;
}

