#include "vicon.h"
//compilation requres -pthread option

int usb_xbee_fd;
int num_bytes_per_read=2;

using namespace std;

//this is ALSO IN CONTROLLER.CPP
//shared_data<Vicon> vicon("v_mem_loc",VICON,"open");

void get_vicon_data(int port, Vicon& vicon_data)
{
     //cout << "entering get_vicon_vicon_data" << endl;
    float data_received[6];
    recieve_data(port, data_received,6);
    unpack_data(vicon_data, data_received);
     // cout << "exit get_vicon_data" << endl;
    
}
Vicon filter_vicon_data(Vicon& new_vicon, Vicon& old_vicon, Vicon& old_old_vicon ,Weights& weights){
    //cout << "filtering vicon data" << endl;

    Vicon filt_vicon = {0.0};

    //filt_vicon.x = (weights.newest * new_vicon.x) + (weights.old * old_vicon.x) + (weights.old_old * old_old_vicon.x);
    filt_vicon.x = filt(new_vicon.x, old_vicon.x, old_old_vicon.x, weights);
    filt_vicon.y = filt(new_vicon.y, old_vicon.y, old_old_vicon.y, weights);
    filt_vicon.z = filt(new_vicon.z, old_vicon.z, old_old_vicon.z, weights);
    filt_vicon.theta = filt(new_vicon.theta, old_vicon.theta, old_old_vicon.theta, weights);
    filt_vicon.phi = filt(new_vicon.phi, old_vicon.phi, old_old_vicon.phi, weights);
    filt_vicon.psi = filt(new_vicon.psi, old_vicon.psi, old_old_vicon.psi, weights);
    
    /*
    cout << "new_vicon.x: " << new_vicon.x << endl;
    cout << "old_vicon.x: " << old_vicon.x << endl;
    cout << "old_old_vicon.x: " << old_old_vicon.x << endl;
    cout << "filt_vicon.x: " << filt_vicon.x << endl;
    
    cout << endl;
     
    cout << "new_vicon.theta: " << new_vicon.theta << endl;
    cout << "old_vicon.theta: " << old_vicon.theta << endl;
    cout << "old_old_vicon.theta: " << old_old_vicon.theta << endl;
    cout << "filt_vicon.theta: " << filt_vicon.theta << endl;

    
    cout << endl;
    */
}
float filt(float new_data, float old_data, float old_old_data, Weights& weights){
    float f = (weights.newest * new_data) + (weights.old * old_data) + (weights.old_old * old_old_data);
    return f;
}
void pushback(Vicon& new_vicon, Vicon& old_vicon, Vicon& old_old_vicon){
    old_old_vicon = old_vicon;
    old_vicon = new_vicon;
}
void unpack_data(Vicon& vicon_data, float arr[]){
    //distributes data from the input buffer to the imu_data data structure
    //we make a char[] to recieve imu data (imu outputs byte by byte)
    //the cast to a float pointer takes the first four bytes in the array 'arr',
    //thus constructing a float
    
//	cout << "enter unpack_data" << endl;
	vicon_data.x =     arr[0];
        vicon_data.y =     arr[1];
        vicon_data.z =     arr[2];
        vicon_data.phi =   arr[3];
        vicon_data.theta = arr[4];
        vicon_data.psi =   arr[5];
//	cout << "exit unpack_data" << endl;
}
void display_vicon_data(const Vicon& vicon_data) {
        printf("<==========================================>\n");   	
       	printf("    VICON DATA    \n");
        printf("phi: %.2f         x: %.2f\n",    vicon_data.phi  ,  vicon_data.x);
        printf("theta: %.2f       y: %.2f\n",    vicon_data.theta,  vicon_data.y);
        printf("psi: %.2f         z: %.2f\n\n\n",vicon_data.psi  ,  vicon_data.z);
}
int open_vicon_port()
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

int recieve_data(int fd_xbee, float data_received[], int data_size)
{

    int bytes_received, attempts, i;
    int MAXATTEMPTS=100;
    uint8_t recv_byte;
    uint8_t recv_buffer[sizeof(float)*data_size+3]; // 1 start byte + data + 2 checksum
    fd_set readfs;
    uint16_t checksum_recv, checksum_calc=0;

    FD_ZERO(&readfs);
    FD_SET(fd_xbee, &readfs);


    // wait for data
    select(fd_xbee+1, &readfs, NULL, NULL, NULL);

    // check the start byte
    read(fd_xbee,&recv_byte,1);

    if(recv_byte==XBEE_START_BYTE)
    {
        // correct start byte

        bytes_received=0;
        attempts=0;

        recv_buffer[bytes_received]=recv_byte;
        bytes_received+=1;

        // receive data
        while(bytes_received < (sizeof(float)*data_size)+3 && attempts++ < MAXATTEMPTS)
        {
            if(read(fd_xbee,&recv_byte,1)==1)
            {
                recv_buffer[bytes_received]=recv_byte;
                bytes_received+=1;
            }
            else
            {
                usleep(1000);
            }

        }

        // Checksum: sum of data without start byte (two bytes: rolls over at 65536)
        for(i=1;i<sizeof(float)*data_size+1;i++)
        {
            checksum_calc+=recv_buffer[i];
        }
        checksum_recv=recv_buffer[sizeof(float)*data_size+1] << 8 | recv_buffer[sizeof(float)*data_size+2]; // high << 8 | low

        if(checksum_recv == checksum_calc)
        {
         // correct checksum
	// convert data to float
        for(i=0;i<data_size;i++) { data_received[i]=*(float *)&recv_buffer[sizeof(float)*i+1]; }
            return bytes_received;
        }
        else
        {
            // incorrect checksum
            printf("checksum_calc:%d, checksum_recv:%d\n",checksum_calc,checksum_recv);
            return -2;
       	}
    }
   else
    {  	// incorrect start byte
        return -1;
    }

 tcflush(fd_xbee,TCIOFLUSH);

}

void vicon_init(void){

     usb_xbee_fd = open_vicon_port();
}
/*
int main(void){

   vicon_init();

   Vicon vicon_data;

    while(1){

  	    cout << "before get_vicon_data" << endl;
            get_vicon_data(usb_xbee_fd, vicon_data);

            //vicon.update(vicon_data); //UNCOMMENT WHEN IMPLEMENTED
  
            display_vicon_data(vicon_data);
     }

    return 0;

}
*/
