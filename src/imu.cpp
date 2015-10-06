//
//  imu.cpp
//  
//
//  Created by Max Wasserman on 9/7/15.
//
//
#include "imu.h"

//shared_data<State> imu("imu_mem_loc",IMU,"create");

int open_imu_port()
{
    //port = open(path, O_RDWR | O_NOCTTY)
    //port        - The returned file handle for the device. -1 if an error occurred
    //path        - The path to the serial port (e.g. /dev/ttyS0)
    //"O_RDWR"    - Opens the port for reading and writing
    //"O_NOCTTY"  - The port never becomes the controlling terminal of the process.
    struct termios newtio;
    
    int port; /* File descriptor for the port */
    port = open(MY_PATH, O_RDWR | O_NOCTTY);
    
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
    newtio.c_cc[VMIN]  = 24;   // block reading until RX x characers. If x = 0, it is non-blocking.
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

void print_data(const State& imu_data)
{
    cout << "theta:      " << imu_data.theta;
    cout << "  phi:    " << imu_data.phi;
    cout << "   psi: " << imu_data.psi << endl;
    cout << "theta_dot: " << imu_data.theta_dot;
    cout << "     phi_dot: " << imu_data.phi_dot;
    cout << "     psi_dot: " << imu_data.psi_dot << endl;
    cout << endl << endl;
}

void unpack_data(State& imu_data, const unsigned char arr[]){
    //distributes data from the input buffer to the imu_data data structure
    //we make a char[] to recieve imu data (imu outputs byte by byte)
    //the cast to a float pointer takes the first four bytes in the array 'arr',
    //thus constructing a float
    
	//cout << "enter unpack_data" << endl;

	imu_data.psi = *(float *)&arr[0];
	imu_data.theta = *(float *)&arr[4];
	imu_data.phi = *(float *)&arr[8];
	imu_data.phi_dot = *(float *)&arr[12];
	imu_data.theta_dot = *(float *)&arr[16];
	imu_data.psi_dot =  *(float *)&arr[20];

	//cout << "exit unpack_data" << endl;
}

void get_imu_data(const int port, State& imu_data)
{
    //cout << "entering get_imu_data" << endl;
    
    unsigned char sensor_bytes2[24] = {0};
    
    //flush input buffer (TCI for input)
    tcflush(port, TCIFLUSH);
    
    //read in 24 bytes of data from port to the address in memory &sensor_bytes2
    //result1 indicates success of reading
    int result = read(port, &sensor_bytes2[0], 24);

    //cout << "number of bytes read into sensor_bytes: " << result << endl;

    //For error message
    if (result == -1){  printf("get_imu_data: FAILED read from port \n");}

    unpack_data(imu_data, sensor_bytes2);
    
   // cout << "exit get_imu_data" << endl;
    
}



/*
int main (void)
{
    // File descriptor for the port 
    int port = open_imu_port();
    //instantiate imu_data ==> scope is over while loop. Same imu_data will be overwritten repeatedly
    State imu_data;
    while(1)
    {   //pull data from sensor and put into imu_data
        get_imu_data(port, imu_data);
        //print_data(imu_data);
	//imu.update(imu_data);  //UNCOMMENT THIS WHEN IMPLEMENTED
    }
    
    
    return 0;
}

*/
