#include <stdlib.h>
#include <stdio.h>
#include "i2c-dev.h"
#include <fcntl.h>
#include <stdint.h>
#include <wiringPi.h>
int main(){
	char filename[20];
	sprintf(filename, "/dev/i2c-1");
	char fd;
	fd = open(filename, O_RDWR);
	if (fd<0){
		printf("Unable to open i2c bus\n");
		exit(1);
	}

	if (ioctl(fd,I2C_SLAVE,0x6b)<0){
	       printf("Error: could not select device\n");
	}	       
	char block[30];
	char blockhigh[30];
        int result;
       	int i;
	int numbytes;
	//i2c_smbus_write_byte_data(fd,0x0A,0x40);
	numbytes = i2c_smbus_read_byte_data(fd, 0x3A);
	numbytes += (i2c_smbus_read_byte_data(fd, 0x3B)&0x0F)<<8;
	printf("num bytes: %d\n",numbytes);
	int status = i2c_smbus_read_byte_data(fd,0x3B)&0x10;
	printf("status: %x\n", status);
	while ((i2c_smbus_read_byte_data(fd, 0x3B) & 0x10) == 0){
		
	//	printf("fifo not empty\n");
		i2c_smbus_read_word_data(fd,0x3E);
	}
	i2c_smbus_write_byte_data(fd,0x0A,0x43);
	while (1){
		i = i2c_smbus_read_byte_data(fd,0x3C);
		printf("\rpattern: %d  ", i);
			//printf("\nAligning\n");
			
		numbytes = i2c_smbus_read_byte_data(fd, 0x3A);
		numbytes += (i2c_smbus_read_byte_data(fd, 0x3B)&0x0F)<<8;
		printf(" unread bytes: %d   | ",numbytes);
		/*while (numbytes < 200){
			printf("\rWaiting... %d                 ", numbytes);
			numbytes=((i2c_smbus_read_byte_data(fd,0x3B)&0x0F)<<8)+i2c_smbus_read_byte_data(fd,0x3A);
			delay(1);
		}*/
		result = i2c_smbus_read_i2c_block_data(fd, 0x3E , sizeof(block), block);
		//result = i2c_smbus_read_i2c_block_data(fd, 0x3F, sizeof(blockhigh), blockhigh);
		//printf("read bytes: %d\n",result);
		//int i;
		/*for (i=0;i<60;i=i+2){
			if ( (i != 0) && (i%6)==0)
				printf("\n");
			printf("%x%x ",block[i],block[i+1]);
		}*/
		uint16_t value = (uint16_t)((block[1]<<8)+(block[0]));
		printf("%x%x: %d ", block[1], block[0], value);
		fflush(stdout);
		numbytes = i2c_smbus_read_byte_data(fd, 0x3A);
		//printf("\nunread bytes: %d\n",numbytes);
		delay(10);
	}

	printf("\n");
	return 0;
}
