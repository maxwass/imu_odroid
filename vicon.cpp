#include "vicon.h"

int onesecond = 1000000;

Positions desired_positions;
Positions START;
Times times;


double delta_position = 0.1;

int usb_xbee;
int bytes_from_vicon = 16;//how many bytes from vicon????

//this is ALSO IN CONTROLLER.CPP
//concur_data<State> state_from_vicon(VICON_MEM_LOC, "vicon");


void *vicon_input(void *thread_id){
	
  State vicon_data;

	while(1){

		get_data(usb_xbee, vicon_data);

		//state_from_vicon.update(raw_vicon_xbee);

		display_info(vicon_data);
	}

}

void get_data(const int port, Vicon& vicon_data)
{
    //cout << "entering get_data" << endl;
    
    unsigned char raw_vicon_xbee[bytes_from_vicon] = {0};
    
    //flush input buffer (TCI for input)
    tcflush(port, TCIFLUSH);
    
    //read in bytes_from_vicon bytes of data from port to the address in memory &raw_vicon_xbee
    //result1 indicates success of reading
    int result = read(port, &raw_vicon_xbee[0], bytes_from_vicon);

    //cout << "number of bytes read into sensor_bytes: " << result << endl;

    //For error message
    if (result == -1){  printf("get_data: FAILED read from port \n");}

    unpack_data(vicon_data, sensor_bytes2);
    
   // cout << "exit get_data" << endl;
    
}
void unpack_data(Vicon& vicon_data, const unsigned char arr[]){
    //distributes data from the input buffer to the imu_data data structure
    //we make a char[] to recieve imu data (imu outputs byte by byte)
    //the cast to a float pointer takes the first four bytes in the array 'arr',
    //thus constructing a float
    
	//cout << "enter unpack_data" << endl;

	vicon_data.x = *(float *)&arr[0];
	vicon_data.y = *(float *)&arr[4];
	vicon_data.z = *(float *)&arr[8];
	vicon_data.phi = *(float *)&arr[12];
	vicon_data.theta = *(float *)&arr[16];
	vicon_data.psi =  *(float *)&arr[20];

	//cout << "exit unpack_data" << endl;
}
void time_calc(Times& times){
	//get current time, swap current and past, calc delta_t
}
void set_desired_positions(Positions& desired_positions){
	desired_positions.x = 0;
	desired_positions.y = 0;
	desired_positions.z = 0;
}
void set_start(Positions& START){
	START.x = 0;
	START.y = 0;
	START.z = 0;
}
void set_initial_times(Times& times){
	times.t_current = 0.0;
	times.t_prev = 0.0;
	times.t_prev_2 = 0.0;
	times.delta_t = 0.0;
}
void configure_threads(void){
	 //configure thread vicon_input

	 pthread_t thread;
     //Special Attribute for starting thread
     pthread_attr_t attr;
     //sched_param is a structure that maintains the scheduling parameters
        //sched_param.sched_priority  - an integer value, the higher the value the higher a thread's proiority for scheduling
     struct sched_param param;
     int fifo_max_prio, fifo_min_prio;
    
     // system("clear"); //NEEDED?????
     //cout << "CALLED SYSTEM('CLEAR') (in vicon) - COULD CAUSE PROBLEMS" << endl;

     cout << "INSIDE CONFIGURE_THREADS (in vicon)" << endl;
    
     gettimeofday(&t_init,NULL);
    
     // Set thread attributes: FIFO scheduling, Joinable
     // the sched_param.sched_priorirty is an int that must be in [min,max] for a certain schedule policy, in this case, SCHED_FIFO
     pthread_attr_init(&attr);
     pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
     fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
     fifo_min_prio = sched_get_priority_min(SCHED_FIFO); 

     cout << "=> creating vicon_input thread" << endl;
     // Medium priority for vicon
     param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&thread, &attr, vicon_input, (void *) 1);
     //sleep just so no undefined behavior if we call join right after create
	 usleep(100);

     pthread_join(thread, NULL);
     close(usb_xbee);
     pthread_attr_destroy(&attr);
}
void display_info(const Vicon& vicon_data) {
        printf("<==========================================>\n");   	
       	printf("    VICON DATA    \n");
        printf("phi: %.2f         x: %.2f\n",    vicon_data.phi  ,  vicon_data.phi);
        printf("theta: %.2f       y: %.2f\n",    vicon_data.theta,  vicon_data.theta);
        printf("psi: %.2f         z: %.2f\n\n\n",vicon_data.psi  ,  vicon_data.psi);
}
void init(void){

	usleep(10000);

	printf("Opening an USB port...   ");//Opens the usb Port
    usb_xbee = open_usbport();
     if (usb_xbee <0)
            printf("\n Error opening an USB0 port!!\n");
         else
            printf("Done!\n");

    set_start(START);
    set_initial_times(times);
	set_desired_positions(desired_positions);

}
int main(void){

	init();

	usleep(onesecond);

	configure_threads();

	return 0;
}



