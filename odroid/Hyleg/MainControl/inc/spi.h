#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sstream>
#include <math.h>
#include "spi_node.h"


int spi_init();
void transfer(int fd, int sel);




