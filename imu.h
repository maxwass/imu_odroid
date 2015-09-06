#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions, Unix API for terminal I/O */


#define BAUDRATE B57600
#define MY_PATH "/dev/ttySAC0"

typedef struct state {
    double theta, phi, psi, theta_dot, phi_dot, psi_dot;
} Sstate ;

typedef struct desired {
    double theta, phi, psi;d
} Sdesired ;

typedef struct control_command {
    double thrust, roll_acc, pitch_acc, yaw_acc;
} Scontrol_command;

typedef struct forces {
    double motor_1, motor_2, motor_3, motor_3;
} Sforces;

using namespace std;

int open_port()
{
    //port = open(path, O_RDWR | O_NOCTTY)
    //port        - The returned file handle for the device. -1 if an error occurred
    //path        - The path to the serial port (e.g. /dev/ttyS0)
    //"O_RDWR"    - Opens the port for reading and writing
    //"O_NOCTTY"  - The port never becomes the controlling terminal of the process.
    struct termios newtio;
    int port; /* File descriptor for the port */
    
    port = open(MY_PATH, O_RDWR | O_NOCTTY);
    
    
    if(port > -1){
        cout << "opened port successfully: " << MY_PATH << endl;
    }
    else{
        cout << "unable to open port: " << MY_PATH << endl;
    }
    
    
    tcgetattr(port, &newtio);
    cfsetospeed(&newtio, BAUDRATE);
   	cfsetispeed(&newtio, BAUDRATE);
    
    //set the number of data bits.
    newtio.c_cflag &= ~CSIZE;  // Mask the character size bits
    newtio.c_cflag |= CS8;
    
    //set the number of stop bits to 1
    newtio.c_cflag &= ~CSTOPB;
    
    //Set parity to None
    newtio.c_cflag &=~PARENB;
    
    //set for non-canonical (raw processing, no echo, etc.)
    newtio.c_iflag = IGNPAR; // ignore parity check close_port(int
    newtio.c_oflag = 0; // raw output
    
    
    //Time-Outs -- won't work with NDELAY option in the call to open
    newtio.c_cc[VMIN]  = 24;   // block reading until RX x characers. If x = 0, it is non-blocking.
    newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s
    
    //Set local mode and enable the receiver
    newtio.c_cflag |= (CLOCAL | CREAD);
    
    //Set the new options for the port...
    int status = tcsetattr(port, TCSANOW, &newtio);
    
    if (status != 0){ //For error message
        printf("Configuring comport failed\n");
        return status;
    }
    
    return port;
}

void print_data(const Sstate  &imu_data)
{
    cout << "theta:      " << imu_data.theta;
    cout << "  phi:    " << imu_data.phi;
    cout << "   psi: " << imu_data.psi << endl;
    cout << "theta_dot: " << imu_data.theta_dot;
    cout << "     phi_dot: " << imu_data.phi_dot;
    cout << "     psi_dot: " << imu_data.psi_dot << endl;
}

void unpack_data(Sstate &imu_data, unsigned char arr[]){
//distributes data from the input buffer to the imu_data data structure
    //we make a char[] to recieve imu data (imu outputs byte by byte)
    //the cast to a float pointer takes the first four bytes in the array 'arr',
    //thus constructing a float
    imu_data.psi = *(float *)&arr[0];
    imu_data.theta = *(float *)&arr[4];
    imu_data.phi = *(float *)&arr[8];
    imu_data.phi_dot = *(float *)&arr[12];
    imu_data.theta_dot = *(float *)&arr[16];
    imu_data.psi_dot =  *(float *)&arr[20];
    
}

void get_data(const int port, Sstate &imu_data)
{
    unsigned char sensor_bytes2[24];
    
    //flush input buffer (TCI for input)
    tcflush(port, TCIFLUSH);
    
    //read in 24 bytes of data from port to the address in memory &sensor_bytes2
    //result1 indicates success of reading
    int result = read(port, &sensor_bytes2[0], 24);
    
    if (result == -1){ //For error message
        printf("get_data: FAILED read from port \n");
    }
    
    unpack_data(imu_data, sensor_bytes2);
}



int main (void)
{
    /* File descriptor for the port */
    int port = open_port(); 
    
    //instantiate imu_data ==> scope is over while loop. Same imu_data will be overwritten repeatedly
    Sstate imu_data;
    while(1)
    {   //pull data from sensor and put into imu_data
        get_data(port, imu_data);
        print_data(imu_data);
    }
    
    
    return 0;
}
