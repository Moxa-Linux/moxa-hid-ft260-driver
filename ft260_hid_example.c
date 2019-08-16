#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
/*
* For the systems that don't have the new version of hidraw.h in userspace.
*/
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len) _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len) _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#define	PCA9535_ADDR_0			0x26
#define	PCA9535_ADDR_1			0x27
#define	PCA9535_MODE_RS232		0x3
#define	PCA9535_MODE_RS485_2W		0x8
#define	PCA9535_MODE_RS422_RS485_4W	0x4

enum uart_mode {
	UART_MODE_RS232 = 0,
	UART_MODE_RS485_2W = 1,
	UART_MODE_RS422_RS485_4W = 2
};


int fd;

const char* bus_type_str(int bus) {
	switch (bus) {
		case BUS_USB: return "USB";
		case BUS_HIL: return "HIL";
		case BUS_BLUETOOTH: return "Bluetooth";
		case BUS_VIRTUAL: return "Virtual";
		default: 		return "Other";
	}
}

int read_reg(unsigned char addr, unsigned char* read_buf) {
	int res;
	unsigned char read_req[5];

	/* For example, if you read Input Port 1, then the next byte read would be Input Port 0. */
	/* There is no limitation on the number of data bytes received in one read transmission */ 
	/* but the final byte received, the bus master must not acknowledge the data. */

	printf("*** Send a read request ***\n");
	printf("addr=0x%x\n", addr);

	/* 1. Send an I2C read request. */
	/* 2. Receive data in one or more I2C input reports.*/
	read_req[0] = 0xC2;	/* I2C read request */ 
	read_req[1] = addr;	/* Slave address */ 
	read_req[2] = 0x06;	/* Start and Stop */ 
	read_req[3] = 0x02;	/* data len, Little Endian */ 
	read_req[4] = 0x00;	/* data len */ 

	res = write(fd, read_req, 5);
	if (res < 0) { 
        	printf("Send an I2C read request error: %d\n", errno);
		return res;
	} else { 
        	printf("Send an I2C read request ok, write() write %d bytes\n", res); 
	}
	sleep(1);

	res = read(fd, read_buf, 4);
	if (res < 0) { 
        	printf("Read Input Report error: %d\n", errno);
		return res;
	} else { 
        	printf("Receive data in one or more I2C input reports ok, read() read %d bytes\n", res); 
	}
	
	printf("Read buffer\n");
	printf("	ReportID:	0x%x\n", read_buf[0]);
	printf("	len:		0x%x\n", read_buf[1]);
	printf("	data 0:		0x%x\n", read_buf[2]);
	printf("	data 1:		0x%x\n", read_buf[3]);

	return res;
}

void write_reg(unsigned char addr, unsigned char* write_regs) {
	int res;
	unsigned char write_buf[7];
	/* After sending data to one register, */
	/* the next data byte is sent to the other register in the pair. 	*/
	/* For example, if the first byte is sent to Output Port 1 (register 3),*/
	/* the next byte is stored in Output Port 0 (register 2).		*/

	printf("*** Send a write request ***\n");

	printf("addr=0x%x\n", addr);
	printf("write_buf[0]=0x%x\n", write_regs[0]);
	printf("write_buf[1]=0x%x\n", write_regs[1]);
	
	/* Set Output Port Registers as RS232/RS422/RS485 mode */
	write_buf[0] = 0xD0;		/* I2C write, 0xD0: maximum data size is 4 bytes */ 
	write_buf[1] = addr;		/* Slave address */ 
	write_buf[2] = 0x06;		/* 0x06: START_AND_STOP */ 
	write_buf[3] = 0x03;		/* data len */ 
	write_buf[4] = 0x02;		/* command byte: Output port 0 is 0x02*/
	write_buf[5] = write_regs[0];	/* Data to Port 0, uart1 ~ uart4 */
	write_buf[6] = write_regs[1];	/* Data to Port 0, uart1 ~ uart4 */

	res = write(fd, write_buf, 7);
	if (res < 0) { 
        	printf("Send a Report Error: %d\n", errno); 
	} else { 
        	printf("Send a Report ok, write() wrote %d bytes\n", res); 
	}
	sleep(1);
}

void write_port0_reg(unsigned char addr, unsigned char write_reg) {
	int res;
	unsigned char write_buf[6];

	printf("*** Send a write request ***\n");
	printf("addr=0x%x\n", addr);
	printf("write_reg=0x%x\n", write_reg);
	
	/* Set Output Port Registers as RS232/RS422/RS485 mode */
	write_buf[0] = 0xD0;		/* I2C write, 0xD0: maximum data size is 4 bytes */ 
	write_buf[1] = addr;		/* Slave address */ 
	write_buf[2] = 0x06;		/* 0x06: START_AND_STOP */ 
	write_buf[3] = 0x02;		/* data len */ 
	write_buf[4] = 0x02;		/* command byte: Output port 0 is 0x02*/
	write_buf[5] = write_reg;	/* Data to Port 0, uart1 ~ uart4 */

	res = write(fd, write_buf, 6);
	if (res < 0) { 
        	printf("Send a Report Error: %d\n", errno); 
	} else { 
        	printf("Send a Report ok, write() wrote %d bytes\n", res); 
	}
	sleep(1);
}

void write_port1_reg(unsigned char addr, unsigned char write_reg) {
	int res;
	unsigned char write_buf[6];

	printf("*** Send a write request ***\n");

	printf("addr=0x%x\n", addr);
	printf("write_reg=0x%x\n", write_reg);
	
	/* Set Output Port Registers as RS232/RS422/RS485 mode */
	write_buf[0] = 0xD0;		/* I2C write, 0xD0: maximum data size is 4 bytes */ 
	write_buf[1] = addr;		/* Slave address */ 
	write_buf[2] = 0x06;		/* 0x06: START_AND_STOP */ 
	write_buf[3] = 0x02;		/* data len */ 
	write_buf[4] = 0x03;		/* command byte: Output port 0 is 0x02*/
	write_buf[5] = write_reg;	/* Data to Port 0, uart1 ~ uart4 */

	res = write(fd, write_buf, 6);
	if (res < 0) { 
        	printf("Send a Report Error: %d\n", errno); 
	} else { 
        	printf("Send a Report ok, write() wrote %d bytes\n", res); 
	}
	sleep(1);
}

void pca9535_init() {
	int res;
	unsigned char write_buf[7];
	unsigned char rs232_reg[2];

	printf("*** Send an initial request ***\n");
	/* Set Configuration Registers as output */
	/* Set chip 0 as output */
	write_buf[0] = 0xD0;	/* I2C write, 0xD0: maximum data size is 4 bytes */ 
	write_buf[1] = 0x26;	/* Slave address */ 
	write_buf[2] = 0x06;	/* 0x06: START_AND_STOP */ 
	write_buf[3] = 0x03;	/* data len */ 
	write_buf[4] = 0x06;	/* Configuration Registers */
	write_buf[5] = 0x00;
 	write_buf[6] = 0x00;

	printf("\n\nSet chip 0 as output\n");
	res = write(fd, write_buf, 7);
	if (res < 0) { 
        	printf("pca9535_init Send a Report Error: %d\n", errno); 
	} else { 
        	printf("pca9535_init Send a Report ok, write() wrote %d bytes\n", res); 
	}
	sleep(1);

	/* Set chip 1 as output */
	write_buf[0] = 0xD0;	/* I2C write, 0xD0: maximum data size is 4 bytes */ 
	write_buf[1] = 0x27;	/* Slave address */ 
	write_buf[2] = 0x06;	/* 0x06: START_AND_STOP */ 
	write_buf[3] = 0x03;	/* data len */ 
	write_buf[4] = 0x06;	/* Configuration Registers */
	write_buf[5] = 0x00;
 	write_buf[6] = 0x00;

	printf("\n\nSet chip 1 as output\n");
	res = write(fd, write_buf, 7);
	if (res < 0) { 
        	printf("pca9535_init Send a Report Error: %d\n", errno); 
	} else { 
        	printf("pca9535_init Send a Report ok, write() wrote %d bytes\n", res); 
	}
	sleep(1);

	/* set RS232 as default mode */
	rs232_reg[0] = 0x33;
	rs232_reg[1] = 0x33;
	write_reg(PCA9535_ADDR_0, rs232_reg);
	write_reg(PCA9535_ADDR_1, rs232_reg);
	/* end of set ports output */
}

int set_uart_mode(int port, int mode, unsigned char addr) {
	int res;	
	unsigned char read_buf[6];
	unsigned char reg_buf[2];
	unsigned char write_buf;
	unsigned char target_mode;

	res = read_reg(addr, read_buf);
	res = read_reg(addr, read_buf);
	if (res < 0) {
		perror("Set uart mode failed");
		return res;
	}

     	printf("high byte = 0x%x\n", read_buf[2]);
     	printf("low byte = 0x%x\n", read_buf[3]);

	printf("*** Set UART #%d to Mode %d ***\n", port, mode);

	/* RS232: LLHH = 0011 = 0x3 */
	/* RS485: LHLL (485-2w) = 0100 = 0x4 */
	/* RS422: HLLL (485-4w) = 1000 = 0x8 */

	switch (mode) {
	case UART_MODE_RS232:
		printf("Set port#%d uart mode to RS232 interface\n", port);
		target_mode = PCA9535_MODE_RS232;
		break;
	case UART_MODE_RS485_2W:
		printf("Set uart#%d mode to RS485-2W interface\n", port);
		target_mode = PCA9535_MODE_RS485_2W;
		break;
	case UART_MODE_RS422_RS485_4W:
		printf("Set uart#%d mode to RS422/RS485-4W interface.\n", port);
		target_mode = PCA9535_MODE_RS422_RS485_4W;
		break;
	default:
		printf("Unknown interface is set.\n");
		return 1;
	}

	port = port % 4;

	switch (port) {
	case 0:
/*
		write_buf = read_buf[2] & 0x0f;
		write_buf |= target_mode << 4;
		printf("Set write_port0_reg as 0x%x\n", write_buf);
		write_port0_reg(addr, write_buf);
*/
		reg_buf[0] = read_buf[2] & 0x0f;
		reg_buf[0] |= target_mode << 4;
		reg_buf[1] = read_buf[3];
		printf("Write regs as 0x%x, 0x%x\n", reg_buf[0], reg_buf[1]);
		write_reg(addr, reg_buf);
		break;
	case 1:
/*
		write_buf = read_buf[2] & 0xf0;
		write_buf |= target_mode;
		printf("Set write_port0_reg as 0x%x\n", write_buf);
		write_port0_reg(addr, write_buf);
*/
		reg_buf[0] = read_buf[2] & 0xf0;
		reg_buf[0] |= target_mode;
		reg_buf[1] = read_buf[3];
		printf("Write regs as 0x%x, 0x%x\n", reg_buf[0], reg_buf[1]);
		write_reg(addr, reg_buf);
		break;
	case 2:
/*
		write_buf = read_buf[3] & 0x0f;
		write_buf |= target_mode << 4;
		printf("Set write_port1_reg as 0x%x\n", write_buf);
		write_port1_reg(addr, write_buf);
*/
		reg_buf[0] = read_buf[2];
		reg_buf[1] = read_buf[3] & 0x0f;
		reg_buf[1] |= target_mode << 4;
		printf("Write regs as 0x%x, 0x%x\n", reg_buf[0], reg_buf[1]);
		write_reg(addr, reg_buf);
		break;
	case 3:
/*
		write_buf = read_buf[3] & 0xf0;
		write_buf |= target_mode;
		printf("Set write_port1_reg as 0x%x\n", write_buf);
		write_port1_reg(addr, write_buf);
*/
		reg_buf[0] = read_buf[2];
		reg_buf[1] = read_buf[3] & 0xf0;
		reg_buf[1] |= target_mode;
		printf("Write regs as 0x%x, 0x%x\n", reg_buf[0], reg_buf[1]);
		write_reg(addr, reg_buf);
		break;

	default:
		printf("Unknown port. Return.\n");
		return 1;
	}
	return 0;
}

int get_uart_mode(int port, unsigned char addr) {
	int res;
	unsigned char read_buf[6];
	unsigned char port_reg;

	res = read_reg(addr, read_buf);
     	printf("high byte = 0x%x\n", read_buf[2]);
     	printf("low byte = 0x%x\n", read_buf[3]);

	port = port % 4;
	printf("port = %d\n", port);

	switch (port) {
	case 0:
		port_reg = read_buf[2] >> 4;
		break;
	case 1:
		port_reg = read_buf[2] & 0xf;
		break;
	case 2:
		port_reg = read_buf[3] >> 4;
		break;
	case 3:
		port_reg = read_buf[3] & 0xf;
		break;
	default:
		printf("Unknown port. Return.\n");
		return 1;
	}
	printf("port_reg = 0x%x\n", port_reg);

	switch (port_reg) {
	case PCA9535_MODE_RS232:
		printf("Get uart mode is RS232 interface.\n");
		return UART_MODE_RS232;
		break;
	case PCA9535_MODE_RS485_2W:
		printf("Get uart mode to RS485 (RS485-2w) interface.\n");
		return UART_MODE_RS485_2W;
		break;
	case PCA9535_MODE_RS422_RS485_4W:
		printf("Get uart mode to RS422 (RS485-4w) interface.\n");
		return UART_MODE_RS422_RS485_4W;
		break;
	default:
		printf("Unknown mode type. Return.\n");
		return -1;
	}

	return res;
}


int main(int argc, char **argv) {
	int res, desc_size = 0;
	unsigned char buf[256];
	unsigned char port_regs[2];
	unsigned char read_buf[6];
	struct hidraw_report_descriptor rpt_desc;
	struct hidraw_devinfo info;
	int port;
	int mode;
	unsigned char chip_addr;

	char* device = "/dev/hidraw0";

	if(argc < 3) {
		printf("Please enter [hiddev] [port] [mode]\n");
		return 1;
	}

	if(argc > 1) {
		device = argv[1];
		port = atoi(argv[2]);
		mode = atoi(argv[3]);
	}

	if((port >= 0) && (port < 4)) {
		chip_addr = PCA9535_ADDR_0;
	} else if((port >= 4) && (port < 8)) {
		chip_addr = PCA9535_ADDR_1;
	} else {
		printf("error port number, return\n");
		return 1;
	}

	printf("you select chip_addr=0x%x\n", chip_addr);
	printf("you select port=%d\n", port);
	printf("you select mode=%d\n", mode);


	/* Open the Device with non-blocking reads. */
	/* It will be better if use libudev instead of hard coded path.
	You can check Appendix A for the example of using libudev */
	fd = open(device, O_RDWR|O_NONBLOCK);
	if (fd < 0) {
		perror("Unable to open device");
		return 1;
	}

	memset(&rpt_desc, 0x0, sizeof(rpt_desc));
	memset(&info, 0x0, sizeof(info));
	memset(buf, 0x0, sizeof(buf));
	memset(port_regs, 0x0, sizeof(port_regs));
	memset(read_buf, 0x0, sizeof(read_buf));

	/* Get Report Descriptor Size */
	res = ioctl(fd, HIDIOCGRDESCSIZE, &desc_size);
	if (res < 0) {
		perror("HIDIOCGRDESCSIZE");
	} else {
		printf("Report Descriptor Size: %d\n", desc_size);
	}

	/* Get Raw Name */
	res = ioctl(fd, HIDIOCGRAWNAME(256), buf);
	if (res < 0) {
		perror("HIDIOCGRAWNAME");
	} else {
		printf("Raw Name: %s\n", buf);
	}

	/* Get Raw Info */
	res = ioctl(fd, HIDIOCGRAWINFO, &info);
	if (res < 0) {
		perror("HIDIOCGRAWINFO");
	} else {
		printf("Raw Info:\n");
		printf("\tbustype: %d (%s)\n",
		info.bustype, bus_type_str(info.bustype));
		printf("\tvendor: 0x%04hx\n", info.vendor);
		printf("\tproduct: 0x%04hx\n", info.product);
	}

#if 0
	/* Set Feature */
	buf[0] = 0xA1; /* SYSTEM_SETTING_ID */
	buf[1] = 0x22; /* I2C_SPEED */
	buf[2] = 0x01; /* 400Kbps */
	buf[3] = 0x90;
	res = ioctl(fd, HIDIOCSFEATURE(4), buf);
	if (res < 0) {
		perror("HIDIOCSFEATURE");
	} else {
		printf("ioctl HIDIOCGFEATURE returned: %d\n", res);
	}
#endif


/*	pca9535_init(); */

	switch (mode) {
	case UART_MODE_RS232:
		printf("Set uart mode to RS232 interface.\n");
		break;
	case UART_MODE_RS485_2W:
		printf("Set uart mode to RS485-2W interface.\n");
		break;
	case UART_MODE_RS422_RS485_4W:
		printf("Set uart mode to RS422/RS485-4W interface.\n");
		break;
	default:
		printf("Unknown interface is set.\n");
		return 1;
	}

/*
	// test
	unsigned char mybuf = 0x48;
	unsigned char target = 0x3;
	unsigned char result = 0x0;

	result = mybuf & 0xf0;
	result |= target;
	printf("result=0x%x\n", result);
*/	


#if 0
	// Wes test Get I2C Status
	buf[0] = 0xC0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = 0;
	res = ioctl(fd, HIDIOCGFEATURE(256), buf);
	if (res < 0) {
		perror("HIDIOCGFEATURE");
	} else {
		printf("ioctl HIDIOCGFEATURE .... returned: %d\n", res);
		printf("    ReportID:   0x%x\n", buf[0]);
		printf("    Bus Status: 0x%x\n", buf[1]);
		printf("    Speed_LSB:  0x%x\n", buf[2]);
		printf("    Speed_MSB:  0x%x\n", buf[3]);
	}
#endif

	get_uart_mode(port, chip_addr);
	set_uart_mode(port, mode, chip_addr);
	get_uart_mode(port, chip_addr);

/*
	port_regs[0] = 0x38;
	port_regs[1] = 0x48;
	write_reg(chip_addr, port_regs);

	read_reg(chip_addr, read_buf);
	port_regs[0] = 0x34;
	port_regs[1] = 0x43;
	write_reg(chip_addr, port_regs);

	read_reg(chip_addr, read_buf);
	read_reg(chip_addr, read_buf);
*/

	close(fd);
	return 0;
}
