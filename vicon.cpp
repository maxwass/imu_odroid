#include "vicon.h"

int onesecond = 1000000;

Positions desired_positions;
Positions START;
Times times;


double delta_position = 0.1;

int usb_xbee;
int num_bytes_from_vicon = 27;

using namespace std;

//this is ALSO IN CONTROLLER.CPP
//concur_data<State> state_from_vicon(VICON_MEM_LOC, "vicon");


void *vicon_input(void *thread_id){
	
  Vicon vicon_data;

	while(1){

		get_data(usb_xbee, vicon_data);

		//state_from_vicon.update(raw_vicon_xbee);

		display_info(vicon_data);
	}

  pthread_exit(NULL);
}

void get_data(const int port, Vicon& vicon_data)
{
    //cout << "entering get_data" << endl;
    
    uint8_t raw_vicon_xbee[num_bytes_from_vicon];// = {0};

    //cout << "allocated raw_vicon_xbee array of size "<< num_bytes_from_vicon  << endl;
    //flush input buffer (TCI for input)
    tcflush(port, TCIFLUSH);
    //cout << "reading data" << endl;
    //read in bytes_from_vicon bytes of data from port to the address in memory &raw_vicon_xbee
    //result1 indicates success of reading
    int result = read(port, &raw_vicon_xbee[0], num_bytes_from_vicon);
    cout << "number of bytes read into sensor_bytes: " << result << endl;
    
    //For error message
    if (result == -1){  printf("get_data: FAILED read from port \n");}
    
    //read first byte and check it
    uint8_t first_byte_recieved = raw_vicon_xbee[0];
    
    //cout << "read data first byte should be 0xFD =? " << raw_vicon_xbee[0] << endl;
    while (first_byte_recieved != 0xFD)
    {
    	cout << "FIRST BYTE INCORRECT, FIRST BYTE INCORRECT, FIRST BYTE INCORRECT, FIRST BYTE INCORRECT" << endl;
	tcflush(port, TCIFLUSH);
    	int result = read(port, &raw_vicon_xbee[0], num_bytes_from_vicon);
   	cout << "number of bytes read into sensor_bytes: " << result << endl;
    	if (result == -1){  printf("get_data: FAILED read from port \n");}
    	//read first byte and check it
    	uint8_t first_byte_recieved = raw_vicon_xbee[0];
    }

    cout<<"FIRST BYTE CORRECT"<<endl;

    unpack_data(vicon_data, raw_vicon_xbee);
    
   // cout << "exit get_data" << endl;
    
}
void unpack_data(Vicon& vicon_data, uint8_t arr[]){
    //distributes data from the input buffer to the imu_data data structure
    //we make a char[] to recieve imu data (imu outputs byte by byte)
    //the cast to a float pointer takes the first four bytes in the array 'arr',
    //thus constructing a float
    
	//cout << "enter unpack_data" << endl;

	vicon_data.x = *(float *)&arr[1];
	vicon_data.y = *(float *)&arr[5];
	vicon_data.z = *(float *)&arr[9];
	vicon_data.phi = *(float *)&arr[13];
	vicon_data.theta = *(float *)&arr[17];
	vicon_data.psi =  *(float *)&arr[21];

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
    
    // gettimeofday(&times.t_current,NULL);
    
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
        printf("phi: %.2f         x: %.2f\n",    vicon_data.phi  ,  vicon_data.x);
        printf("theta: %.2f       y: %.2f\n",    vicon_data.theta,  vicon_data.y);
        printf("psi: %.2f         z: %.2f\n\n\n",vicon_data.psi  ,  vicon_data.z);
}
int open_port()
{
    //port = open(path, O_RDWR | O_NOCTTY)
    //port        - The returned file handle for the device. -1 if an error occurred
    //path        - The path to the serial port (e.g. /dev/ttyS0)
    //"O_RDWR"    - Opens the port for reading and writing
    //"O_NOCTTY"  - The port never becomes the controlling terminal of the process.
    struct termios newtio;
    
    int port; /* File descriptor for the port */
    string MY_PATH = "/dev/ttyUSB0";
    port = open(MY_PATH.c_str(), O_RDWR | O_NOCTTY);
    printf("Opening an USB port...   ");//Opens the usb Port

    //open_usbport  -- this is in reciever.h
    //port = open_usbport();  //this opens port and only waits until it gets 2 bytes!!!!
    if(port > -1){ cout << "opened port successfully: " << MY_PATH << endl; }
    else         { cout << "unable to open port: "      << MY_PATH << endl; }
    
    //sets the parameters associated with the terminal
    //from the termios structure referred to by newtio.
    tcgetattr(port, &newtio);

    //set input/output baudrate
    cfsetospeed(&newtio, BAUDRATE);
    cfsetispeed(&newtio, BAUDRATE);
    
    //set character size mask.
    //set the number of data bits.
   
    //CSIZE flag is a mask that specifies the number of bits per byte for both transmission 
    //and reception. This size does not include the parity bit, if any. The values for the 
    //field defined by this mask are CS5, CS6, CS7, and CS8, for 5, 6, 7,and 8 bits per byte, respectively
    newtio.c_cflag &= ~CSIZE; 
    newtio.c_cflag |= CS8;     //8 bits/byte
    
    //set the number of stop bits to 1
    newtio.c_cflag &= ~CSTOPB;
    
    //Set parity to None
    newtio.c_cflag &=~PARENB;
    
    //set for non-canonical (raw processing, no echo, etc.)
    newtio.c_iflag = IGNPAR; // ignore parity check close_port(int
    newtio.c_oflag = 0; // raw output
    newtio.c_lflag = 0; // raw input (this puts us in non-canonical mode!)
    
    
    //Time-Outs -- won't work with NDELAY option in the call to open
    //Will read until recieved a minimum of 24 bytes, no time limit
    newtio.c_cc[VMIN]  = num_bytes_from_vicon;   // block reading until RX x characers. If x = 0, it is non-blocking.
    newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s
    
    //Set local mode and enable the receiver
    newtio.c_cflag |= (CLOCAL | CREAD);
    
    //Set the new options for the port...
    //TCSANOW - options go into affect immediately
    int status = tcsetattr(port, TCSANOW, &newtio);
    
    if (status != 0){ //For error message
        printf("Configuring comport failed\n");
        return status;
    }
    return port;
}
void init(void){

	usleep(10000);

	printf("Opening an USB port...   ");//Opens the usb Port
    usb_xbee = open_port();

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



