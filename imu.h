#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>

//things to check: baudrate on this and imu, path to serial port

//baud rate is rate of info transfer, ensure this is the same as the imu's == 57600
#define BAUDRATE B115200

typedef struct imu_data {
double theta, phi, psi, theta_dot, phi_dot, psi_dot;
} SImu_data ;

int open_port()
{
    //termios - Unix API for terminal I/O
    
    //port = open(path, O_RDWR | O_NOCTTY)
        //port        - The returned file handle for the device. -1 if an error occurred
        //path        - The path to the serial port (e.g. /dev/ttyS0)
        //"O_RDWR"    - Opens the port for reading and writing
        //"O_NOCTTY"  - The port never becomes the controlling terminal of the process.
    struct termios newtio;
	int port;

	port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
/* Daewon suggested this is not needed: try commenting out
//	tcgetattr(port, &newtio);
//	cfsetospeed(&newtio, BAUDRATE);
//   	cfsetispeed(&newtio, BAUDRATE);
//    
//       //set the number of data bits.
//       newtio.c_cflag &= ~CSIZE;  // Mask the character size bits
//       newtio.c_cflag |= CS8;
//    
//       //set the number of stop bits to 1
//       newtio.c_cflag &= ~CSTOPB;
//    
//       //Set parity to None
//       newtio.c_cflag &=~PARENB;
//    
//       //set for non-canonical (raw processing, no echo, etc.)
//       newtio.c_iflag = IGNPAR; // ignore parity check close_port(int
//       newtio.c_oflag = 0; // raw output
//       newtio.c_lflag = 0; // raw input
*/
    
    //ask daewon for further clarificaton on newtios
    
       //Time-Outs -- won't work with NDELAY option in the call to open
       newtio.c_cc[VMIN]  = 24;   // block reading until RX x characers. If x = 0, it is non-blocking.
       newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s
    
/* Daewon suggested this is not needed: try commenting out
        //Set local mode and enable the receiver
       newtio.c_cflag |= (CLOCAL | CREAD);
    
       //tcflush(port, TCIFLUSH);
	//Set the new options for the port...
	int status=tcsetattr(port, TCSANOW, &newtio);
	
	if (status != 0){ //For error message
		printf("Configuring comport failed\n");
		return status;
	}
*/
    
	return port;
}

