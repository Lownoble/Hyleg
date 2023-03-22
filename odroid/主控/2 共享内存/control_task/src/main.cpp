 
#include <sys/shm.h>  
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <time.h> 
#include "comm.h"
#include "spi_node.h"
#include "sys_time.h"
#include "include.h"
#define MEM_SPI 1111
#define MEM_CONTROL 1112
#define MEM_SIZE 2048  

_SPI_RX spi_rx;
_SPI_TX spi_tx;  

_NAV_RX nav_rx;
_NAV_TX nav_tx;
int mem_connect=0;
float mem_loss_cnt=0;
int mem_connect_c=0;
float mem_loss_cnt_c=0;
struct shareMemory 
{  
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];  
};  
struct shareMemory shareMemory_spi;
 
unsigned char mem_read_buf[MEM_SIZE];  
unsigned char mem_write_buf[MEM_SIZE];  
int mem_read_cnt=0;
int mem_write_cnt=0;
 
static void setDataFloat_mem(float f, int *anal_cnt)
{
	int i = *(int *)&f;
	mem_write_buf[*anal_cnt++] = ((i << 24) >> 24);
	mem_write_buf[*anal_cnt++] = ((i << 16) >> 24);
	mem_write_buf[*anal_cnt++] = ((i << 8) >> 24);
	mem_write_buf[*anal_cnt++] = (i >> 24);
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

float rx_test[2]={0};
void memory_read(void){//读取内存SERVO
char temp;
mem_read_cnt=0;
rx_test[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
rx_test[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
printf("rx:: %f %f\n",rx_test[0] ,rx_test[1] );
}

void memory_write(void)//写入内存SERVO spi
{
 static float temp=0;
 mem_write_cnt=MEM_SIZE/2;

setDataFloat_mem( 3);
setDataFloat_mem( 4);
}

int main()  
{  
    float timer[5]={0};
    float sys_dt = 0,dT=0;
    int i=0;
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功  
    void *shm_rx = shmat(shmid_rx, 0, 0);  
    shareMemory *pshm_rx = (shareMemory*)shm_rx;  
    printf("Memory SPI attached at %p\n",shm_rx);

    while(1)  
    {  
       sys_dt = Get_Cycle_T(0);
	   timer[0]+=sys_dt;
    
//---------------------------------------------------------------------//
       //共享内存读取 to Servo task
        if(pshm_rx->flag == 1)  
        {  
            if(!mem_connect){
			printf("MEM Servo CONNECT!!!\n");
			mem_connect=1;
			}
			mem_loss_cnt=0;

            for(int k=0;k<MEM_SIZE/2-1;k++)
				mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            memory_write();
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
				pshm_rx->szMsg[k]=mem_write_buf[k];		
            
            pshm_rx->flag = 0;  
        }  

        mem_loss_cnt+=sys_dt;
        if(mem_loss_cnt>1&&mem_connect==1){
			mem_connect=0;
			mem_loss_cnt=0;
			for (int i = 0; i < 4; i++){
			spi_tx.q_set[i][0] = spi_rx.q[i][0];
			spi_tx.q_set[i][1] = spi_rx.q[i][1];
			spi_tx.tau_ff[i][0] = 0;
			spi_tx.tau_ff[i][1] = 0;
			}
			spi_tx.kp= spi_tx.ki= spi_tx.kd= spi_tx.en_motor= 0;

			printf("MEM Servo LOSS!!!\n");
		}	
        usleep(100);
    }  
      
    shmdt(shm_rx);  //失败返回-1，假设成功 
    shmctl(shmid_rx, IPC_RMID, 0);  //失败返回-1，假设成功。仅在reader这里删除共享内存，保证读完最后一个消息
    return 0;  
} 