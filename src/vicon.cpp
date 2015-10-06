#include "vicon.h"

//compilation requres -pthread option
int onesecond = 1000000;

int usb_xbee;
int num_bytes_per_read=2;

using namespace std;

//this is ALSO IN CONTROLLER.CPP
//shared_data<Vicon> vicon("v_mem_loc",VICON,"open");

void get_vicon_data(int port, Vicon& vicon_data)
{
    //cout << "entering get_vicon_vicon_data" << endl;
 
    float data_received[6];

    XBee_receive_float(usb_xbee, data_received,6);
    unpack_data(vicon_data, data_received);
    
   // cout << "exit get_vicon_data" << endl;
    
}
void unpack_data(Vicon& vicon_data, float arr[]){
    //distributes data from the input buffer to the imu_data data structure
    //we make a char[] to recieve imu data (imu outputs byte by byte)
    //the cast to a float pointer takes the first four bytes in the array 'arr',
    //thus constructing a float
    
	cout << "enter unpack_data" << endl;
	vicon_data.x =     arr[0];
        vicon_data.y =     arr[1];
        vicon_data.z =     arr[2];
        vicon_data.phi =   arr[3];
        vicon_data.theta = arr[4];
        vicon_data.psi =   arr[5];
	cout << "exit unpack_data" << endl;
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
    
    //gets the parameters (options) associated with the terminal from the termios structure (newtio)
    tcgetattr(port, &newtio);

    //Set local mode and enable the receiver
    newtio.c_cflag |= (CLOCAL | CREAD);

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
    
    //no parity bits
    newtio.c_cflag &=~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    
    //set for non-canonical (raw processing, no echo, etc.)
    newtio.c_iflag = IGNPAR; // ignore parity errors
    newtio.c_oflag = 0; // raw output
    
    //Time-Outs -- won't work with NDELAY option in the call to open
    //Will read until recieved a minimum of num_bytes_per_read bytes, no time limit
    newtio.c_cc[VMIN]  = num_bytes_per_read;   // block reading until RX x characers. If x = 0, it is non-blocking.
    newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s
    
    
    //Set the new options for the port...
    //TCSANOW - options go into affect immediately
    int status = tcsetattr(port, TCSANOW, &newtio);
    
    if (status != 0){ //For error message
        printf("Configuring Vicon comport failed\n");
        return status;
    }
    return port;
}
void init(void){

     usb_xbee = open_port();
}
int main(void){

    init();

  //  usleep(onesecond);

    Vicon vicon_data;

    while(1){

  	    cout << "before get_vicon_data" << endl;
            get_vicon_data(usb_xbee, vicon_data);

            //vicon.update(vicon_data); //UNCOMMENT WHEN IMPLEMENTED
  
            display_info(vicon_data);
     }

    return 0;

}



