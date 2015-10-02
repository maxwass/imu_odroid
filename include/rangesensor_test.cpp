#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>
#include <termios.h>
#define LIDAR_LITE_ADRS 0x62

#define MEASURE_VAL 0x04
#define MEASURE_REG 0x00
#define STATUS_REG  0x47
#define DISTANCE_REG_HI 0x0f
#define DISTANCE_REG_LO 0x10  
#define VERSION_REG 0x41

#define ERROR_READ -1

// Status Bits
#define STAT_BUSY               0x01
#define STAT_REF_OVER           0x02
#define STAT_SIG_OVER           0x04
#define STAT_PIN                0x08
#define STAT_SECOND_PEAK        0x10
#define STAT_TIME               0x20
#define STAT_INVALID            0x40
#define STAT_EYE                0x80
#define DISTANCE_CONT		0x8f
int i2cHandle;

struct timeval t_init, t_now, t_now_v, t_before;
double t_v, del_t_v, t_landing, t_prev_v = 0.0;
double t_prev = 0.0;
double t, del_t;

char sprintf_buffer[10000000];
int sprintf_buffer_loc = 0;

int Get_Range_Values(){
	char range_read_lo[2] = { 0 };
	char range_read_hi[2] = { 0 };
	unsigned char sensor_read_hi;
	unsigned char sensor_read_lo;
	unsigned char write_buffer[2];
	unsigned char write_buffer2[2];
	unsigned char Write_buf_st;
	unsigned char st;

	write_buffer[0] = MEASURE_REG;
	write_buffer[1] = MEASURE_VAL;
	write_buffer2[0] = DISTANCE_REG_LO;
	write_buffer2[1] = DISTANCE_REG_HI;
	Write_buf_st = STATUS_REG;

	int hiVal, loVal, i=0, res, LoVal;
	res = write(i2cHandle, &write_buffer[0], 2);
	if (res < 0) printf("Error writing REG\n");
	usleep(5000);
	
	
	res = write(i2cHandle, &write_buffer2[0] , 1);
	if (res < 0) printf("Error writing Lo\n");
	usleep(500);
	res = read(i2cHandle, range_read_lo,1);
	if (res < 0) printf("Error reading Lo1\n");
	usleep(5000);
	
	res = write(i2cHandle, &write_buffer2[1] , 1);
	if (res < 0) printf("Error writing hi\n");
	usleep(500);
	res = read(i2cHandle, range_read_hi,1);
	if (res < 0) printf("Error reading hi1\n");
	usleep(5000);

	return( (range_read_hi[0] << 8) + range_read_lo[0]);
	

}

int main(void)
{
int range_res;
 gettimeofday(&t_init,NULL);
  FILE *file;
 file = fopen("data.txt","w");
	printf("opening i2c port...\n");
 i2cHandle = open("/dev/i2c-4",O_RDWR);

 if (i2cHandle > 0)
	printf("Done!\n");
 else
	printf("Fail to open i2c port!\n");
 usleep(1000000);

 
 while(t<20.){
  ioctl(i2cHandle,I2C_SLAVE,LIDAR_LITE_ADRS);
        gettimeofday(&t_now,NULL);
        t = (t_now.tv_sec - t_init.tv_sec) ;
        t += (t_now.tv_usec - t_init.tv_usec)/1000000.;
        
        del_t=t-t_prev;
        t_prev=t;
 	range_res = Get_Range_Values();
 	printf("%3.0d cm   Hz: %e\n", range_res, 1/del_t);
 	if(sprintf_buffer_loc < sizeof(sprintf_buffer))
        {
           int sprintf_size = sprintf(sprintf_buffer+sprintf_buffer_loc,"%E %d\n", t, range_res);

            sprintf_buffer_loc+=sprintf_size;
        }
       else if(sprintf_buffer_loc >= sizeof(sprintf_buffer))
        {
            printf("Warning: sprintf_buffer is full! \n");
        }
 }
    printf("Opening text file.....  ");
    printf("Saving buffer to a file.....  ");
    fwrite(sprintf_buffer, 1, sprintf_buffer_loc,file);
    printf("free buffer file.....  ");
    fclose(file);
    close(i2cHandle);
 return 0;
}

