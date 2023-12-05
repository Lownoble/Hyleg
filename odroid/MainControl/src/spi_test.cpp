/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */
 
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
 
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
 
static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 1000000;
static uint16_t delay;

#define SPI_BUF_SIZE 256 
uint8_t spi_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t spi_rx_buf[SPI_BUF_SIZE] = {0};
int spi_tx_cnt = 0;
int spi_rx_cnt = 0;

static void setDataInt_spi(int i)
{
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi(float f)
{
	int i = *(int *)&f;
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static float floatFromData_spi(unsigned char *data, int *anal_cnt)
{
	int i = 0x00;
	i |= (*(data + *anal_cnt + 3) << 24);
	i |= (*(data + *anal_cnt + 2) << 16);
	i |= (*(data + *anal_cnt + 1) << 8);
	i |= (*(data + *anal_cnt + 0));

	*anal_cnt += 4;
	return *(float *)&i;
}

static char charFromData_spi(unsigned char *data, int *anal_cnt)
{
	int temp = *anal_cnt;
	*anal_cnt += 1;
	return *(data + temp);
}

static int intFromData_spi(unsigned char *data, int *anal_cnt)
{
	int i = 0x00;
	i |= (*(data + *anal_cnt + 3) << 24);
	i |= (*(data + *anal_cnt + 2) << 16);
	i |= (*(data + *anal_cnt + 1) << 8);
	i |= (*(data + *anal_cnt + 0));
	*anal_cnt += 4;
	return i;
}


void can_board_send(int sel)//发送到单片机
{
	int i;
	static float t_temp = 0;
	char id = 0;
	char sum_t = 0, _cnt = 0;
	spi_tx_cnt = 0;

	spi_tx_buf[spi_tx_cnt++] = 0xFA;
	spi_tx_buf[spi_tx_cnt++] = 0xFF;
	spi_tx_buf[spi_tx_cnt++] = sel;
	spi_tx_buf[spi_tx_cnt++] = 0;

	switch (sel)
	{
	case 0:
		setDataInt_spi(0);
		setDataFloat_spi(3.14);
		setDataFloat_spi(2.22);
		setDataFloat_spi(38.15);
		setDataFloat_spi(38.15);
		setDataFloat_spi(38.15);
		break;


	case 98:// FA FF 62 08 0F 01 00 00 9C 00 00 00 0F 00 00
		setDataInt_spi(271); 
		setDataInt_spi(156); 
		break;

	default:
		for (int id = 0; id < 4; id++)
		{
			setDataFloat_spi(0); 
			setDataFloat_spi(0);
			setDataFloat_spi(0);
			setDataFloat_spi(0);
		}
		break;
	}

	spi_tx_buf[3] = (spi_tx_cnt)-4;
	for (i = 0; i < spi_tx_cnt; i++)
		sum_t += spi_tx_buf[i];
	spi_tx_buf[spi_tx_cnt++] = sum_t;

	for(int i=0;i<3;i++){
		spi_tx_buf[spi_tx_cnt++] = 0;
	}

}

static void transfer(int fd)
{
	can_board_send(0);
	int ret;
	uint8_t tx[] = {
		0xfa, 0xff, 0x04, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00,
	};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr;
	
	memset(&tr, 0, sizeof(tr));
	tr.tx_buf = (unsigned long)spi_tx_buf;
	tr.rx_buf = (unsigned long)rx;
	tr.len = spi_tx_cnt;
	tr.delay_usecs = delay;
	tr.speed_hz = speed;
	tr.bits_per_word = bits;
	tr.cs_change = 0;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
 
	for (ret = 0; ret < spi_tx_cnt; ret++) {
		printf("%.2X ", rx[ret]);
	}
	puts("");
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}
 
static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 0, 0, 0 },
		};
		int c;
 
		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);
 
		if (c == -1)
			break;
 
		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}
 
int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;
 
	parse_opts(argc, argv);
 
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");
 
	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");
 
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
 
	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");
 
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");
 
	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");
 
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");
 
	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	while(1){
		transfer(fd);
		usleep(500);
	}

 
	close(fd);
 
	return ret;
}
