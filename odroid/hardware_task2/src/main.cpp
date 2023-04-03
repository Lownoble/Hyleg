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
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "comm.h"
#include "spi_node.h"
#include "spi.h"
#include "sys_time.h"
#include <pthread.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>

YAML::Node config_hardware=YAML::LoadFile("/home/odroid/Corgi/Param/param_hardware.yaml");

// ps -ef | grep hardware_task
// kill －9 324
_MEMS mems;

#define SPI_TEST 0

#define USE_USB 0
#define USE_SERIAL 0

#define EN_SPI_BIG 1
#define CAN_LINK_COMM_VER1 0
#define CAN_LINK_COMM_VER2 1//3 BLDC Param DIV
#if EN_SPI_BIG
#if CAN_LINK_COMM_VER1
    #define SPI_SEND_MAX  85
#else
    #define SPI_SEND_MAX  120+20+20//equal to stm32 spi send cnt
#endif
#else
#define SPI_SEND_MAX  40//40
#endif
#define EN_MULTI_THREAD 1
#define NO_THREAD 0
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MEM_SPI 0001
#define MEM_SIZE 2048
#define EN_DBG_PRINT 0

#if NO_THREAD&&!EN_MULTI_THREAD
static uint32_t speed = 150000*4;//3Mhz 死机
#define DELAY_SPI 250//us
#else
static uint32_t speed = 20000000;//odroid 20M  upbord 2M  raspberrypi 5M
#define DELAY_SPI 250//us
#endif
float spi_loss_cnt = 0;
int spi_connect = 0;
float mems_usb_loss_cnt = 0;
int mems_usb_connect = 0;

using namespace std;

_SPI_RX spi_rx;
_SPI_TX spi_tx;

uint8_t spi_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t spi_rx_buf[SPI_BUF_SIZE] = {0};
int spi_tx_cnt_show=0;
int spi_tx_cnt = 0;
int spi_rx_cnt = 0;

uint8_t usb_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t usb_rx_buf[SPI_BUF_SIZE] = {0};
int usb_tx_cnt = 0;
int usb_rx_cnt = 0;

uint8_t tx[SPI_BUF_SIZE] = {};
uint8_t rx[ARRAY_SIZE(tx)] = {};


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

static void setDataFloat_spi_int(float f,float size)
{
    int16_t _temp;
    _temp=f*size;
    spi_tx_buf[spi_tx_cnt++] = BYTE1(_temp);
    spi_tx_buf[spi_tx_cnt++] = BYTE0(_temp);
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

static float floatFromData_spi_int(unsigned char *data, int *anal_cnt,float size)
{
    float temp=0;
    temp=(float)((int16_t)(*(data + *anal_cnt + 0)<<8)|*(data + *anal_cnt + 1))/size;
    *anal_cnt += 2;
    return temp;
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

float To_180_degrees(float x)
{
    return (x>180?(x-360):(x<-180?(x+360):x));
}

void can_board_send(char sel)//发送到单片机
{
    int i;
    static float t_temp = 0;
    char id = 0;
    char sum_t = 0, _cnt = 0;
    static char bldc_id_sel=0;
    spi_tx_cnt = 0;

    spi_tx_buf[spi_tx_cnt++] = 0xFE;
    spi_tx_buf[spi_tx_cnt++] = 0xFC;
    spi_tx_buf[spi_tx_cnt++] = sel;
    spi_tx_buf[spi_tx_cnt++] = 0;

    switch (sel)
    {
    case 41://2022/4/4 3BLDC param div---------------------------------
        for (int id = 0; id < 4; id++)
        {
            setDataFloat_spi_int(spi_tx.q_set[id][0],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.q_set[id][1],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.q_set[id][2],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.tau_ff[id][0],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.tau_ff[id][1],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.tau_ff[id][2],CAN_T_DIV);
            setDataFloat_spi_int(spi_tx.q_reset[id][0],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.q_reset[id][1],CAN_POS_DIV);
            setDataFloat_spi_int(spi_tx.q_reset[id][2],CAN_POS_DIV);
            spi_tx_buf[spi_tx_cnt++] =spi_tx.param_sel[id];//chose sw or st param
        }
            setDataFloat_spi_int(spi_tx.t_to_i,1000);
#if 1
            setDataFloat_spi_int(spi_tx.kp,CAN_GAIN_DIV_P);
            setDataFloat_spi_int(spi_tx.ki,CAN_GAIN_DIV_I);
            setDataFloat_spi_int(spi_tx.kd,CAN_GAIN_DIV_D);

            setDataFloat_spi_int(spi_tx.kp_sw_d[bldc_id_sel],CAN_GAIN_DIV_P);
            setDataFloat_spi_int(spi_tx.ki_sw_d[bldc_id_sel],CAN_GAIN_DIV_I);
            setDataFloat_spi_int(spi_tx.kd_sw_d[bldc_id_sel],CAN_GAIN_DIV_D);

            setDataFloat_spi_int(spi_tx.kp_st_d[bldc_id_sel],CAN_GAIN_DIV_P);
            setDataFloat_spi_int(spi_tx.ki_st_d[bldc_id_sel],CAN_GAIN_DIV_I);
            setDataFloat_spi_int(spi_tx.kd_st_d[bldc_id_sel],CAN_GAIN_DIV_D);
#endif
            //printf("bldc_id_sel=%d %f %f %f\n",bldc_id_sel,spi_tx.kp_sw_d[bldc_id_sel],spi_tx.ki_sw_d[bldc_id_sel],spi_tx.kd_sw_d[bldc_id_sel]);

            spi_tx_buf[spi_tx_cnt++] = (spi_tx.max_i);
            spi_tx_buf[spi_tx_cnt++] = spi_tx.en_motor*100+spi_tx.reser_q*10+spi_tx.reset_err;
            spi_tx_buf[spi_tx_cnt++] = bldc_id_sel*100+spi_tx.led_enable[0]*10+spi_tx.led_enable[1];//bldc id sel
//OCU param
            setDataFloat_spi_int( mems.imu_att.z,CAN_POS_DIV);
            spi_tx_buf[spi_tx_cnt++] =  mems.Acc_CALIBRATE*100+mems.Gyro_CALIBRATE*10+mems.Mag_CALIBRATE;
            spi_tx_buf[spi_tx_cnt++] =  spi_tx.beep_state;

            setDataFloat_spi_int(spi_tx.arm_cmd_s.pos_set.x,20);
            setDataFloat_spi_int(spi_tx.arm_cmd_s.pos_set.y,20);
            setDataFloat_spi_int(spi_tx.arm_cmd_s.pos_set.z,20);
            setDataFloat_spi_int(spi_tx.arm_cmd_s.att_set[0],20);
            setDataFloat_spi_int(spi_tx.arm_cmd_s.att_set[1],20);
            setDataFloat_spi_int(spi_tx.arm_cmd_s.att_set[2],20);
            spi_tx_buf[spi_tx_cnt++] = spi_tx.arm_cmd_s.power*10+spi_tx.arm_cmd_s.mode;
            spi_tx_buf[spi_tx_cnt++] = spi_tx.arm_cmd_s.cap;

            bldc_id_sel++;
            if(bldc_id_sel>2)
                bldc_id_sel=0;
    break;

    case 50://发送OCU配置
        //--------------------Param From OCU-------------
        setDataFloat_spi_int( mems.imu_pos.x,1000);
        setDataFloat_spi_int( mems.imu_pos.y,1000);
        setDataFloat_spi_int( mems.imu_pos.z,1000);
        setDataFloat_spi_int( mems.imu_att.x,CAN_POS_DIV);
        setDataFloat_spi_int( mems.imu_att.y,CAN_POS_DIV);
        setDataFloat_spi_int( mems.imu_att.z,CAN_POS_DIV);
        setDataFloat_spi_int( mems.gps_pos.x,1000);
        setDataFloat_spi_int( mems.gps_pos.y,1000);
        setDataFloat_spi_int( mems.gps_pos.z,1000);
        spi_tx_buf[spi_tx_cnt++] =  mems.Acc_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] =  mems.Gyro_CALIBRATE;
        spi_tx_buf[spi_tx_cnt++] =  mems.Mag_CALIBRATE;

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

    if(spi_tx_cnt>SPI_SEND_MAX)
       printf("spi_tx_cnt=%d over flow!!!\n",spi_tx_cnt);
    spi_tx_cnt_show=spi_tx_cnt;
}

int slave_rx(uint8_t *data_buf, int num)//接收解码--------------from stm32
{
    static int cnt_p = 0;
    static int cnt_err_sum=0;
    uint8_t id;
    uint8_t sum = 0;
    uint8_t i;
    uint8_t temp;
    int anal_cnt = 4;
    for (i = 0; i < (num - 1); i++)
        sum += *(data_buf + i);
    if (!(sum == *(data_buf + num - 1))){
        printf("spi sum err=%d sum_cal=0x%X sum=0x%X !!\n",cnt_err_sum++,sum,*(data_buf + num - 1));
        return 0;
    }
    if (!(*(data_buf) == 0xFF && *(data_buf + 1) == 0xFB)){
        printf("spi head err!!\n");
        return 0;
    }
    if (*(data_buf + 2) == 24) //------------------old version---------- use now=======================2022/4/16
    {
        spi_loss_cnt = 0;
        if (spi_connect==0)
        {
            printf("Hardware::Hardware SPI-STM32 Link3-Sbus Yunzhuo!!!=%d!!!\n",spi_connect);
            spi_connect = 1;
        }

        spi_rx.att[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        //printf("att0=%f att1=%f att2=%f dt=%f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2], Get_Cycle_T(0));
        spi_rx.att_rate[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att_rate[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.att_rate[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);

        spi_rx.acc_b[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.acc_b[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.acc_b[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);

        spi_rx.acc_n[0] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.acc_n[1] = floatFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.acc_n[2] = floatFromData_spi(spi_rx_buf, &anal_cnt);

        for (int i = 0; i < 4; i++)
        {
            spi_rx.q[i][0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_POS_DIV);
            spi_rx.q[i][1] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_POS_DIV);
            spi_rx.q[i][2] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_POS_DIV);
            spi_rx.tau[i][0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);
            spi_rx.tau[i][1] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);
            spi_rx.tau[i][2] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,CAN_T_DIV);

            temp= charFromData_spi(spi_rx_buf, &anal_cnt);
            spi_rx.connect[i] = temp/100;
            spi_rx.connect_motor[i][0] = (temp-spi_rx.connect[i] *100)/10;
            spi_rx.ready[i][0] = temp%10;
            temp= charFromData_spi(spi_rx_buf, &anal_cnt);
            spi_rx.connect[i] = temp/100;
            spi_rx.connect_motor[i][1] = (temp-spi_rx.connect[i] *100)/10;
            spi_rx.ready[i][1] = temp%10;
            temp= charFromData_spi(spi_rx_buf, &anal_cnt);
            spi_rx.connect[i] = temp/100;
            spi_rx.connect_motor[i][2] = (temp-spi_rx.connect[i] *100)/10;
            spi_rx.ready[i][2] = temp%10;
            //printf("%d %d\n", spi_rx.connect_motor[i][2], spi_rx.ready[i][2] );
        }

        spi_rx.bat_v[0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.bat_v[0] =spi_rx.bat_v[1] =spi_rx.bat_v[2];

        //Sbus from STM32
        spi_rx.ocu.sbus_ch[0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[1] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[2] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[3] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[4] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.ocu.sbus_ch[5] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);

        temp= charFromData_spi(spi_rx_buf, &anal_cnt);
        spi_rx.ocu.sbus_conncect = temp/100;
        spi_rx.ocu.sbus_aux[0] = (temp-int(temp/100) *100)/10;
        spi_rx.ocu.sbus_aux[1] = temp%10;

        temp= charFromData_spi(spi_rx_buf, &anal_cnt);
       // spi_rx.ocu.sbus_aux[0] = temp/100;
        spi_rx.ocu.sbus_aux[2] = (temp-int(temp/100) *100)/10;
        spi_rx.ocu.sbus_aux[3] = temp%10;

        temp= charFromData_spi(spi_rx_buf, &anal_cnt);
       // spi_rx.ocu.sbus_aux[0] = temp/100;
        spi_rx.ocu.sbus_aux[4] = (temp-int(temp/100) *100)/10;
        spi_rx.ocu.sbus_aux[5] = temp%10;

        spi_rx.aoa.dis= floatFromData_spi_int(spi_rx_buf, &anal_cnt,100);
        spi_rx.aoa.angle= floatFromData_spi_int(spi_rx_buf, &anal_cnt,10);
        spi_rx.aoa.rssi= charFromData_spi(spi_rx_buf, &anal_cnt);

        //--arm
        spi_rx.arm_cmd_s.pos_now.x = floatFromData_spi_int(spi_rx_buf, &anal_cnt,20);
        spi_rx.arm_cmd_s.pos_now.y = floatFromData_spi_int(spi_rx_buf, &anal_cnt,20);
        spi_rx.arm_cmd_s.pos_now.z = floatFromData_spi_int(spi_rx_buf, &anal_cnt,20);
        spi_rx.arm_cmd_s.att_now[0] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,20);
        spi_rx.arm_cmd_s.att_now[1] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,20);
        spi_rx.arm_cmd_s.att_now[2] = floatFromData_spi_int(spi_rx_buf, &anal_cnt,20);
         #if 0
        printf("%f %f %d\n",spi_rx.aoa.dis,spi_rx.aoa.angle,spi_rx.aoa.rssi);
        #endif
        #if 0
          printf("connect=%d st=%d back=%d x=%d a=%d\n",
              spi_rx.ocu.connect ,  spi_rx.ocu.key_st,spi_rx.ocu.key_back, spi_rx.ocu.key_x,spi_rx.ocu.key_a );
          printf("b=%d y=%d ll=%d rr=%d lr=%d up=%d\n",
              spi_rx.ocu.key_b ,  spi_rx.ocu.key_y,spi_rx.ocu.key_ll, spi_rx.ocu.key_rr,spi_rx.ocu.key_lr ,spi_rx.ocu.key_ud);
          printf("spd0=%f spd1=%f att0=%f att1=%f yaw=%f\n",
              spi_rx.ocu.rc_spd_w[0],spi_rx.ocu.rc_spd_w[1],spi_rx.ocu.rc_att_w[0],spi_rx.ocu.rc_att_w[1],spi_rx.ocu.rate_yaw_w);
        #endif
        #if 0
          printf("connect=%d st=%d back=%d x=%d a=%d\n",
              spi_rx.ocu.sbus_conncect ,  spi_rx.ocu.sbus_aux[0],spi_rx.ocu.sbus_aux[1], spi_rx.ocu.sbus_aux[2],spi_rx.ocu.sbus_aux[3] );
          printf("rc0=%f rc1=%f rc2=%f rc3=%f\n",
              spi_rx.ocu.sbus_ch[0],spi_rx.ocu.sbus_ch[1],spi_rx.ocu.sbus_ch[2],spi_rx.ocu.sbus_ch[3]);
        #endif
    }
    return 1;
}

void transfer(int fd, int sel)//发送
{
    static uint8_t state, rx_cnt;
    static uint8_t _data_len2 = 0, _data_cnt2 = 0;
    int ret;
    uint8_t data = 0;

    can_board_send(sel);
    //printf("before1\n");
    ret=SPIDataRW(0,spi_tx_buf,rx,SPI_SEND_MAX); //向总线中写入7个数据
    //printf("after1\n");//printf("ret=%d cnt=%d\n",ret,spi_tx_cnt);
    #if 0
    for(int i=0;i<ret;i++)
        printf("0x%X ",rx[i]);

    printf("ret=%d \n",ret);
    #endif
    if (ret < 1){
       printf("SPI Reopen!\n");
       SPISetup(0,speed);//printf("can't send spi message\n");
    }
    else
    {
        //printf("ret=%d\n",ret);
        for (int i = 0; i < SPI_SEND_MAX; i++)
        {
            data = rx[i];
            if (state == 0 && data == 0xFF)
            {
                state = 1;
                spi_rx_buf[0] = data;
            }
            else if (state == 1 && data == 0xFB)
            {
                state = 2;
                spi_rx_buf[1] = data;
            }
            else if (state == 2 && data > 0 && data < 0XF1)
            {
                state = 3;
                spi_rx_buf[2] = data;
            }
            else if (state == 3 && data < SPI_BUF_SIZE)
            {
                state = 4;
                spi_rx_buf[3] = data;
                _data_len2 = data;
                _data_cnt2 = 0;
            }
            else if (state == 4 && _data_len2 > 0)
            {
                _data_len2--;
                spi_rx_buf[4 + _data_cnt2++] = data;
                if (_data_len2 == 0)
                    state = 5;
            }
            else if (state == 5)
            {
                state = 0;
                spi_rx_buf[4 + _data_cnt2] = data;
                spi_rx_cnt = 4;
                slave_rx(spi_rx_buf, _data_cnt2 + 5);
            }
            else
                state = 0;
            //printf("%02x ",rx[i]);
        }
        //printf("\n");
    }
    //printf("after2\n");
}

//----------------------------------------------------------------------------
int mem_connect=0;
float mem_loss_cnt=0;
struct shareMemory
{
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];
};
struct shareMemory shareMemory_spi;

unsigned char mem_write_buf[MEM_SIZE];
unsigned char mem_read_buf[MEM_SIZE];
int mem_write_cnt=0;
static void setDataFloat_mem(float f)
{
    int i = *(int *)&f;
    mem_write_buf[mem_write_cnt++] = ((i << 24) >> 24);
    mem_write_buf[mem_write_cnt++] = ((i << 16) >> 24);
    mem_write_buf[mem_write_cnt++] = ((i << 8) >> 24);
    mem_write_buf[mem_write_cnt++] = (i >> 24);
}

static void setDataChar_mem(char f)
{
    mem_write_buf[mem_write_cnt++] = (f);
}

void memory_write(void)//写入内存 to control
{
    static float temp=0;
    mem_write_cnt=0;
    setDataFloat_mem( spi_rx.att[0]);
    setDataFloat_mem( spi_rx.att[1]);
    setDataFloat_mem( spi_rx.att[2]);
    setDataFloat_mem( spi_rx.att_rate[0]);
    setDataFloat_mem( spi_rx.att_rate[1]);
    setDataFloat_mem( spi_rx.att_rate[2]);
    setDataFloat_mem( spi_rx.acc_b[0]);
    setDataFloat_mem( spi_rx.acc_b[1]);
    setDataFloat_mem( spi_rx.acc_b[2]);
    setDataFloat_mem( spi_rx.acc_n[0]);
    setDataFloat_mem( spi_rx.acc_n[1]);
    setDataFloat_mem( spi_rx.acc_n[2]);
    //-----------MEMS USB

    //printf("moco:%f %f %f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2]);
    //printf("moco:%f %f %f\n",spi_rx.acc_b[0],spi_rx.acc_b[1],spi_rx.acc_b[2]);
    spi_rx.att_usb[0]=-AHRSData_Packet.Roll*57.3;
    spi_rx.att_usb[1]= AHRSData_Packet.Pitch*57.3;
    spi_rx.att_usb[2]= To_180_degrees(AHRSData_Packet.Heading*57.3);
#if 0
    spi_rx.att_rate_usb[0]=-AHRSData_Packet.RollSpeed*57.3;
    spi_rx.att_rate_usb[1]= AHRSData_Packet.PitchSpeed*57.3;
    spi_rx.att_rate_usb[2]=-AHRSData_Packet.HeadingSpeed*57.3;
#else
    spi_rx.att_rate_usb[0]=-IMUData_Packet.gyroscope_x*57.3;
    spi_rx.att_rate_usb[1]= IMUData_Packet.gyroscope_y*57.3;
    spi_rx.att_rate_usb[2]=-IMUData_Packet.gyroscope_z*57.3;
   // printf("moco:%f %f %f\n",spi_rx.att_rate[0],spi_rx.att_rate[1],spi_rx.att_rate[2]);
   // printf("use::%f %f %f\n",spi_rx.att_rate_usb[0],spi_rx.att_rate_usb[1],spi_rx.att_rate_usb[2]);
#endif
    spi_rx.acc_b_usb[0]=-IMUData_Packet.accelerometer_x/9.81;
    spi_rx.acc_b_usb[1]= IMUData_Packet.accelerometer_y/9.81;
    spi_rx.acc_b_usb[2]=-IMUData_Packet.accelerometer_z/9.81;
    //printf("use::%f %f %f\n",spi_rx.att_usb[0],spi_rx.att_usb[1],spi_rx.att_usb[2]);
    //printf("use::%f %f %f\n",spi_rx.acc_b_usb[0],spi_rx.acc_b_usb[1],spi_rx.acc_b_usb[2]);

    setDataChar_mem (mems_usb_connect);
    setDataFloat_mem( To_180_degrees(spi_rx.att_usb[0]-spi_rx.att_usb_bias[0])*mems_usb_connect);
    setDataFloat_mem( To_180_degrees(spi_rx.att_usb[1]-spi_rx.att_usb_bias[1])*mems_usb_connect);
    setDataFloat_mem( To_180_degrees(spi_rx.att_usb[2]-spi_rx.att_usb_bias[2])*mems_usb_connect);
    setDataFloat_mem( (spi_rx.att_rate_usb[0]-spi_rx.att_rate_usb_bias[0])*mems_usb_connect);
    setDataFloat_mem( (spi_rx.att_rate_usb[1]-spi_rx.att_rate_usb_bias[1])*mems_usb_connect);
    setDataFloat_mem( (spi_rx.att_rate_usb[2]-spi_rx.att_rate_usb_bias[2])*mems_usb_connect);
    setDataFloat_mem( spi_rx.acc_b_usb[0]*mems_usb_connect);
    setDataFloat_mem( spi_rx.acc_b_usb[1]*mems_usb_connect);
    setDataFloat_mem( spi_rx.acc_b_usb[2]*mems_usb_connect);

    for (int i = 0; i < 4; i++)
    {
        setDataFloat_mem(spi_rx.q[i][0]);
        setDataFloat_mem(spi_rx.q[i][1]);
        setDataFloat_mem(spi_rx.q[i][2]);
        setDataFloat_mem(spi_rx.tau[i][0]);
        setDataFloat_mem(spi_rx.tau[i][1]);
        setDataFloat_mem(spi_rx.tau[i][2]);

        setDataFloat_mem(spi_rx.bat_v[i]);

        setDataChar_mem(spi_rx.connect[i]*100+spi_rx.connect_motor[i][0]*10+spi_rx.ready[i][0]);
        setDataChar_mem(spi_rx.connect[i]*100+spi_rx.connect_motor[i][1]*10+spi_rx.ready[i][1]);
        setDataChar_mem(spi_rx.connect[i]*100+spi_rx.connect_motor[i][2]*10+spi_rx.ready[i][2]);
    }

    //SBUS
    setDataChar_mem( spi_rx.ocu.sbus_conncect *100+ spi_rx.ocu.sbus_aux[0]*10+spi_rx.ocu.sbus_aux[1]);
    setDataChar_mem( spi_rx.ocu.sbus_aux[2]*10+spi_rx.ocu.sbus_aux[3]);
    setDataChar_mem( spi_rx.ocu.sbus_aux[4]*10+spi_rx.ocu.sbus_aux[5]);

    setDataFloat_mem(spi_rx.ocu.sbus_ch[0]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[1]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[2]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[3]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[4]);
    setDataFloat_mem(spi_rx.ocu.sbus_ch[5]);

    //AOA UWB
    setDataFloat_mem(spi_rx.aoa.angle);
    setDataFloat_mem(spi_rx.aoa.dis);
    setDataFloat_mem(spi_rx.aoa.rssi);

    //arm
    setDataFloat_mem(spi_rx.arm_cmd_s.pos_now.x);
    setDataFloat_mem(spi_rx.arm_cmd_s.pos_now.y);
    setDataFloat_mem(spi_rx.arm_cmd_s.pos_now.z);
    setDataFloat_mem(spi_rx.arm_cmd_s.att_now[0]);
    setDataFloat_mem(spi_rx.arm_cmd_s.att_now[1]);
    setDataFloat_mem(spi_rx.arm_cmd_s.att_now[2]);

}

void memory_read(void){//读取内存 from control
int mem_read_cnt=MEM_SIZE/2;
float test1,test2;
for (int i = 0; i < 4; i++)
{
    spi_tx.q_set[i][0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.q_set[i][1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.q_set[i][2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.q_reset[i][0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.q_reset[i][1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.q_reset[i][2] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.tau_ff[i][0] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.tau_ff[i][1] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.tau_ff[i][2] = floatFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;

    spi_tx.param_sel[i]=mem_read_buf[mem_read_cnt++];//id isolate
}
    spi_tx.t_to_i= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.max_i= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_sw= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_sw= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_sw= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_st= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_st= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_st= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    //bldc 0
    spi_tx.kp_sw_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_sw_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_sw_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_st_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_st_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_st_d[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    //bldc 1
    spi_tx.kp_sw_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_sw_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_sw_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_st_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_st_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_st_d[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    //bldc 2
    spi_tx.kp_sw_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_sw_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_sw_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.kp_st_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.ki_st_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.kd_st_d[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);

 //   printf("%f %f %f\n",spi_tx.q_reset[0][0],spi_tx.q_reset[0][1],spi_tx.q_reset[0][2]);
//    printf("%f %f %f %f %f %f\n",spi_tx.kp_sw,spi_tx.ki_sw,spi_tx.kd_sw,
//           spi_tx.kp_st,spi_tx.ki_st,spi_tx.kd_st);
    spi_tx.en_motor= mem_read_buf[mem_read_cnt++]*mem_connect;
    spi_tx.reser_q= mem_read_buf[mem_read_cnt++];
    spi_tx.reset_err= mem_read_buf[mem_read_cnt++];

    spi_tx.led_enable[0]= mem_read_buf[mem_read_cnt]/10; spi_tx.led_enable[1]= mem_read_buf[mem_read_cnt++]%10;

    //-----------------------OCU param-------------------
    mems.imu_pos.x= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_pos.y= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_pos.z= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_att.x= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_att.y= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.imu_att.z= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.gps_pos.x= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.gps_pos.y= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.gps_pos.z= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    mems.Acc_CALIBRATE=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    mems.Gyro_CALIBRATE=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    mems.Mag_CALIBRATE=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.beep_state=charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;

    //arm
    spi_tx.arm_cmd_s.pos_set.x= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_cmd_s.pos_set.y= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_cmd_s.pos_set.z= floatFromData_spi(mem_read_buf, &mem_read_cnt);

    spi_tx.arm_cmd_s.att_set[0]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_cmd_s.att_set[1]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_cmd_s.att_set[2]= floatFromData_spi(mem_read_buf, &mem_read_cnt);
    spi_tx.arm_cmd_s.power= charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.arm_cmd_s.mode= charFromData_spi(mem_read_buf, &mem_read_cnt)*mem_connect;
    spi_tx.arm_cmd_s.cap= charFromData_spi(mem_read_buf, &mem_read_cnt);
}


pthread_mutex_t lock;

void* Thread_Mem(void*)//内存管理线程
{
    static int cnt = 0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int i=0,memory_update=0;
    int flag = 0;
    int link_cnt=0;
    //共享内存
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    pshm_rx->flag = 0;
    printf("Hardware::Memory Hardware attached at %p\n",shm_rx);

    while (1)
    {
        //共享内存写
        if(pshm_rx->flag == 0)
        {
            if(!mem_connect){
            mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("Hardware::Memery Control Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;
            //pthread_mutex_lock(&lock);
            memory_write();
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            //pthread_mutex_unlock(&lock);
            pshm_rx->flag = 1;
        }else{
            mem_loss_cnt+=sys_dt;
            mem_init_cnt=0;
        }

        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            for (int i = 0; i < 4; i++){
            spi_tx.q_set[i][0] = spi_rx.q[i][0];
            spi_tx.q_set[i][1] = spi_rx.q[i][1];
            spi_tx.q_set[i][2] = spi_rx.q[i][2];
            spi_tx.tau_ff[i][0] = 0;
            spi_tx.tau_ff[i][1] = 0;
            spi_tx.tau_ff[i][2] = 0;
            }
            spi_tx.kp= spi_tx.ki= spi_tx.kd= spi_tx.en_motor= 0;
            spi_tx.kp_sw= spi_tx.ki_sw= spi_tx.kd_sw= spi_tx.en_motor= 0;
            spi_tx.kp_st= spi_tx.ki_st= spi_tx.kd_st= spi_tx.en_motor= 0;
            printf("Hardware::Memery Control Loss!!!\n");
        }

        usleep(500);
    }
    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //
    return 0;
}


void* Thread_SPI(void*)//内存管理线程
{
    static float timer_spi1 = 0,timer_spi2=0;
    static int cnt = 0;
    static int timer_1s=0;
    static int timer_1m=0;
    static int timer_1h=0;
    static float timer_cnt=0;
    float sys_dt = 0;
    int i=0,memory_update=0;
    int flag = 0;
    int fd = 0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();
    fd=SPISetup(0,speed); //初始化SPI通道0，并设置为最大速度500000hz
    if(fd==-1)
        printf("init spi failed!\n");

    while (1)
    {
        sys_dt = Get_Cycle_T(15);
        timer_cnt+=sys_dt;
        if(timer_cnt>1){
            timer_cnt=0;
            timer_1s++;
            if(timer_1s>60){timer_1s=0;
                timer_1m++;
            }
            if(timer_1m>60){timer_1m=0;
                timer_1h++;}

            printf("Hardware::SPI Still Online at hour-%d min-%d sec-%d spi_cnt=%d\n",timer_1h,timer_1m,timer_1s,spi_tx_cnt_show);
        }

        spi_loss_cnt += sys_dt;
        if (spi_loss_cnt > 1.5&& spi_connect==1)
        {
            spi_loss_cnt = 0;
            spi_connect = 0;
            printf("Hardware::Hardware SPI-STM32 Loss!!!\n");
        }
        //-------SPI CAN发送
        timer_spi1+= sys_dt;
        timer_spi2+= sys_dt;

        transfer(fd, 41);//3 bldc div

        usleep(DELAY_SPI);
    }
    close(fd);
    return 0;
}


#if NO_THREAD&&!EN_MULTI_THREAD
int Thread_ALL(void)
#else
void* Thread_ALL(void*)
#endif
{
    static float timer_spi1 = 0,timer_spi2=0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int i=0,memory_update=0;
    int flag = 0;
    int link_cnt=0;
    int fd = 0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();

    fd=SPISetup(0,speed); //初始化SPI通道0，并设置为最大速度500000hz
    if(fd==-1)
        printf("init spi failed!\n");
    //while(1);
    //共享内存
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    pshm_rx->flag = 0;
    printf("Hardware::Memory Hardware attached at %p\n",shm_rx);
    while (1)
    {
        sys_dt = Get_Cycle_T(15);
        spi_loss_cnt += sys_dt;
        if (spi_loss_cnt > 1.5&& spi_connect==1)
        {
            spi_loss_cnt = 0;
            spi_connect = 0;
            printf("Hardware::Hardware SPI-STM32 Loss!!!\n");
        }
        //-------SPI CAN发送
        timer_spi1+= sys_dt;
        timer_spi2+= sys_dt;

        transfer(fd, 41);

        //共享内存写
        if(pshm_rx->flag == 0)
        {
            if(!mem_connect){
                mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("Hardware::Memery Control Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;
            memory_write();
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            pshm_rx->flag = 1;
        }else
            mem_loss_cnt+=sys_dt;

        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            for (int i = 0; i < 4; i++){
            spi_tx.q_set[i][0] = spi_rx.q[i][0];
            spi_tx.q_set[i][1] = spi_rx.q[i][1];
            spi_tx.q_set[i][2] = spi_rx.q[i][2];
            spi_tx.tau_ff[i][0] = 0;
            spi_tx.tau_ff[i][1] = 0;
            spi_tx.tau_ff[i][2] = 0;
            }
            spi_tx.kp= spi_tx.ki= spi_tx.kd= spi_tx.en_motor= 0;
            spi_tx.kp_st= spi_tx.ki_st= spi_tx.kd_st= spi_tx.en_motor= 0;
            spi_tx.kp_sw= spi_tx.ki_sw= spi_tx.kd_sw= spi_tx.en_motor= 0;
            printf("Hardware::Memery Control Loss!!!\n");
        }
        usleep(200);
    }
    close(fd);
    shmdt(shm_rx);  //失败返回-1，假设成功
    shmctl(shmid_rx, IPC_RMID, 0);  //
    return 0;
}


//===========================================================USB-STM32 Comm===========================
#include "serial.hpp"
#include <serial/serial.h>
#include <iostream>
uint8_t buff[512];

using namespace std;
#if USE_SERIAL
//serial::Serial m_serial("/dev/ttyACM0",2000000 , serial::Timeout::simpleTimeout(1000));
#endif
const char *dev  = "/dev/ttyACM0";

void* Thread_USB(void*)//not use now
{

    int i;
    static uint8_t state_usb=0, rx_cnt_usb=0;
    static uint8_t _data_len2_usb = 0, _data_cnt2_usb = 0;
    int ret=0;
    int rx_length=42;
    uint8_t data_usb = 0;
    int iResult = -1;
    int fd = -1,iCommPort,iBaudRate,iDataSize,iStopBit;
    char cParity;
    int iLen;
#if !USE_SERIAL
    iCommPort = 1;
    fd = open_port(iCommPort);
    if( fd<0 )
    {
    close_port(fd);
    printf("Hardware::USB open_port error !\n");
    //return 1;
    }
    iBaudRate = 2000000;
    iDataSize = 8;
    cParity = 'N';
    iStopBit = 1;
    iResult = set_port(fd,iBaudRate,iDataSize,cParity,iStopBit);
#else
    serial::Serial m_serial;
    //配置串口：serial_port_ = "/dev/ttyUSB0"
    m_serial.setPort( "/dev/ttyACM0");
    //配置波特率：baudrate_ = 115200
    m_serial.setBaudrate(2000000);
    m_serial.setBytesize(serial::eightbits);
    //设置奇偶校验位//serial::parity_even :偶校验  serial::parity_odd :奇校验 serial::parity_mark :校验位始终为1 serial::parity_space :校验位始终为0 serial::parity_none :无校验
    m_serial.setParity(serial::parity_none);
    //设置停止位 serial::stopbits_one ：1位 serial::stopbits_one_point_five ：1.5位 serial::stopbits_two ：2位
    m_serial.setStopbits(serial::stopbits_one);
    m_serial.open();
    cout << "Is the serial port open?"<< endl;
    if(m_serial.isOpen())
    cout << " Yes." << endl;
    else
    cout << " No." << endl;
#endif
//------------------------------------------memery-------------------------------------
    static float timer_spi1 = 0,timer_spi2=0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int memory_update=0;
    int flag = 0;
    int link_cnt=0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();
    //共享内存
#if 0
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功
    shareMemory *pshm_rx = (shareMemory*)shm_rx;
    pshm_rx->flag = 0;
    printf("Hardware::Memory Hardware attached at %p\n",shm_rx);
#endif
    fd_set  rset;
    int  rv = -1 ;
    int nread=0;
    struct timeval timeout;
    timeout.tv_sec=0;
    timeout.tv_usec=1000;

    while (1)
    {

        sys_dt = Get_Cycle_T(20);
        spi_loss_cnt += sys_dt;
        if (spi_loss_cnt > 1.5&& spi_connect==1)
        {
            spi_loss_cnt = 0;
            spi_connect = 0;
            printf("Hardware::Hardware USB-STM32 Loss!!!\n");
        }

#if USE_SERIAL
        nread = m_serial.available();
        // printf(" read_cefore=%d\n",nread);
        if (nread>0){
            printf(" \n read_cefore=%d\n",nread);
            m_serial.read(buff, nread);
            //m_serial.flush();
            //m_serial.flushInput();
        }
#else
        //iLen = read_port(fd,buff,rx_length);
        FD_ZERO(&rset);
        FD_SET(fd, &rset);
        rv = select(fd+1, &rset, NULL, NULL, &timeout);
        nread=0;
        if(rv < 0)
        {
            //printf("select() failed: %s\n", strerror(errno));
            //return 0;
        }else{
        nread = read(fd, buff, rx_length);
        //printf("nread=%d\n",nread);
        }
#endif
       // printf("nread=%d\n ",nread);
        for (int i = 0; i < nread; i++)
        {
            //data_usb = buff[i];
            printf("%02x ",buff[i]);
            // printf("%d ",buff[i]);
            if (state_usb == 0 && data_usb == 0xFF)
            {
                state_usb = 1;
                usb_rx_buf[0] = data_usb;
            }
            else if (state_usb == 1 && data_usb == 0xFB)
            {
                state_usb = 2;
                usb_rx_buf[1] = data_usb;
                //printf("rx_check1\n");
            }
            else if (state_usb == 2 && data_usb > 0 && data_usb < 0XF1)
            {
                state_usb = 3;
                usb_rx_buf[2] = data_usb;
                 //printf("rx_check2 data=%d\n",data_usb);
            }
            else if (state_usb == 3 && data_usb < SPI_BUF_SIZE)
            {
                state_usb = 4;
                usb_rx_buf[3] = data_usb;
                _data_len2_usb = data_usb;
                _data_cnt2_usb = 0;
                //printf("rx_check3 data=%d\n",data_usb);
            }
            else if (state_usb == 4 && _data_len2_usb > 0)
            {
                _data_len2_usb--;
                usb_rx_buf[4 + _data_cnt2_usb++] = data_usb;
                if (_data_len2_usb == 0)
                    state_usb = 5;
            }
            else if (state_usb == 5)
            {
                state_usb = 0;
                usb_rx_buf[4 + _data_cnt2_usb] = data_usb;
                usb_rx_cnt = 4;
                for(i=0;i<SPI_BUF_SIZE;i++)
                    spi_rx_buf[i]=usb_rx_buf[i];
                slave_rx(spi_rx_buf, _data_cnt2_usb + 5);
                //printf("att0=%f att1=%f att2=%f dt=%f\n",spi_rx.att[0],spi_rx.att[1],spi_rx.att[2], Get_Cycle_T(0));
                //printf("rx_check3\n");
            }
            else
                state_usb = 0;
        }

       // printf(" iLen=%d\n",iLen);
        //printf("\n");

        //-------USB CAN发送
        timer_spi1+= sys_dt;
        timer_spi2+= sys_dt;
        if (timer_spi2 > 1)
        {
            timer_spi2 = 0;
            can_board_send(50);
        }
        #if EN_SPI_BIG||0
            can_board_send(20);
        #else
        else if (timer_spi1 > 0.1)//10ms
        {
            timer_spi1 = 0;
            can_board_send(2);
        }
        else
            can_board_send(1); //1ms
        #endif
#if 1
        for(i=0;i<spi_tx_cnt;i++){
            usb_tx_buf[i]=spi_tx_buf[i];
            //printf("%02x ",usb_tx_buf[i]);
        }
       // printf("\n");
#else
        spi_tx_cnt=0;
        for(i=0;i<41;i++)
         spi_tx_buf[spi_tx_cnt++]=i;
#endif
#if !USE_SERIAL
        iLen = write_port(fd,usb_tx_buf,spi_tx_cnt);
#else
        m_serial.write(usb_tx_buf, spi_tx_cnt);
#endif
        //printf("tx=%d\n",iLen);
#if 0
        //共享内存写
        if(pshm_rx->flag == 0)
        {
            if(!mem_connect){
                mem_init_cnt++;
                if(mem_init_cnt>3){
                printf("Hardware::Memery Control Link=%d!!!\n",link_cnt++);
                mem_connect=1;
                }
            }
            mem_loss_cnt=0;
            memory_write();
            for(int k=0;k<MEM_SIZE/2-1;k++)
                pshm_rx->szMsg[k]=mem_write_buf[k];
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
                mem_read_buf[k]=pshm_rx->szMsg[k];
            memory_read();
            pshm_rx->flag = 1;
        }else
            mem_loss_cnt+=sys_dt;

        if(mem_loss_cnt>1&&mem_connect==1){
            mem_connect=0;
            mem_loss_cnt=0;
            mem_init_cnt=0;
            for (int i = 0; i < 4; i++){
            spi_tx.q_set[i][0] = spi_rx.q[i][0];
            spi_tx.q_set[i][1] = spi_rx.q[i][1];
            spi_tx.tau_ff[i][0] = 0;
            spi_tx.tau_ff[i][1] = 0;
            }
            spi_tx.kp= spi_tx.ki= spi_tx.kd= spi_tx.en_motor= 0;

            printf("Hardware::Memery Control Loss!!!\n");
        }
#endif
       usleep(50);
    }
    close(fd);
   // shmdt(shm_rx);  //失败返回-1，假设成功
    //shmctl(shmid_rx, IPC_RMID, 0);  //
    return 0;
}


//===========================================================USB-MEMS==========================================================
IMUData_Packet_t IMUData_Packet;
AHRSData_Packet_t AHRSData_Packet;
char ttl_receive;
char Fd_data[64];
char Fd_rsimu[64];
char Fd_rsahrs[64];
int rs_imutype =0;
int rs_ahrstype =0;
extern int Time_count;
const char *dev_usb_mems  = "/dev/ttyUSB0";

/*************
实现16进制的can数据转换成浮点型数据
****************/
float DATA_Trans(char Data_1,char Data_2,char Data_3,char Data_4)
{
    long long transition_32;
    float tmp=0;
    int sign=0;
    int exponent=0;
    float mantissa=0;
    transition_32 = 0;
    transition_32 |=  Data_4<<24;
    transition_32 |=  Data_3<<16;
    transition_32 |=  Data_2<<8;
    transition_32 |=  Data_1;
    sign = (transition_32 & 0x80000000) ? -1 : 1;//符号位
    //先右移操作，再按位与计算，出来结果是30到23位对应的e
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    //将22~0转化为10进制，得到对应的x系数
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    tmp=sign * mantissa * pow(2, exponent);
    return tmp;
}

long long timestamp(char Data_1,char Data_2,char Data_3,char Data_4)
{
    long transition_32;
    transition_32 = 0;
    transition_32 |=  Data_4<<24;
    transition_32 |=  Data_3<<16;
    transition_32 |=  Data_2<<8;
    transition_32 |=  Data_1;
    return transition_32;
}

void AHRSData2PC(void)
{
    printf("AHRS: The RollSpeed =  %f\r\n",AHRSData_Packet.RollSpeed);
    printf("AHRS: The PitchSpeed =  %f\r\n",AHRSData_Packet.PitchSpeed);
    printf("AHRS: The HeadingSpeed =  %f\r\n",AHRSData_Packet.HeadingSpeed);
    printf("AHRS: The Roll =  %f\r\n",AHRSData_Packet.Roll);
    printf("AHRS: The Pitch =  %f\r\n",AHRSData_Packet.Pitch);
    printf("AHRS: The Heading =  %f\r\n",AHRSData_Packet.Heading);
//    printf("AHRS: The Quaternion.Qw =  %f\r\n",AHRSData_Packet.Qw);
//    printf("AHRS: The Quaternion.Qx =  %f\r\n",AHRSData_Packet.Qx);
//    printf("AHRS: The Quaternion.Qy =  %f\r\n",AHRSData_Packet.Qy);
//    printf("AHRS: The Quaternion.Qz =  %f\r\n",AHRSData_Packet.Qz);
//    printf("AHRS: The Timestamp =  %d\r\n",AHRSData_Packet.Timestamp);
}

void IMUData2PC(void)
{
    //printf("Now start sending IMU data.\r\n");
    printf("IMU: The gyroscope_x =  %f\r\n",IMUData_Packet.gyroscope_x);
    printf("IMU:The gyroscope_y =  %f\r\n",IMUData_Packet.gyroscope_y);
    printf("IMU:The gyroscope_z =  %f\r\n",IMUData_Packet.gyroscope_z);
    printf("IMU:The accelerometer_x =  %f\r\n",IMUData_Packet.accelerometer_x);
    printf("IMU:The accelerometer_y =  %f\r\n",IMUData_Packet.accelerometer_y);
    printf("IMU:The accelerometer_z =  %f\r\n",IMUData_Packet.accelerometer_z);
//    printf("IMU:The magnetometer_x =  %f\r\n",IMUData_Packet.magnetometer_x);
//    printf("IMU:The magnetometer_y =  %f\r\n",IMUData_Packet.magnetometer_y);
//    printf("IMU:The magnetometer_z =  %f\r\n",IMUData_Packet.magnetometer_z);
//    printf("IMU:The Timestamp =  %d\r\n",IMUData_Packet.Timestamp);
    //printf("Now the data of IMU has been sent.\r\n");

}

char TTL_Hex2Dec(void)
{
    char i;
     if(rs_ahrstype==1)
    {
        if(Fd_rsahrs[1]==TYPE_AHRS&&Fd_rsahrs[2]==AHRS_LEN)
        {
        AHRSData_Packet.RollSpeed=DATA_Trans(Fd_rsahrs[7],Fd_rsahrs[8],Fd_rsahrs[9],Fd_rsahrs[10]);       //横滚角速度
        AHRSData_Packet.PitchSpeed=DATA_Trans(Fd_rsahrs[11],Fd_rsahrs[12],Fd_rsahrs[13],Fd_rsahrs[14]);   //俯仰角速度
        AHRSData_Packet.HeadingSpeed=DATA_Trans(Fd_rsahrs[15],Fd_rsahrs[16],Fd_rsahrs[17],Fd_rsahrs[18]); //偏航角速度

        AHRSData_Packet.Roll=DATA_Trans(Fd_rsahrs[19],Fd_rsahrs[20],Fd_rsahrs[21],Fd_rsahrs[22]);      //横滚角
        AHRSData_Packet.Pitch=DATA_Trans(Fd_rsahrs[23],Fd_rsahrs[24],Fd_rsahrs[25],Fd_rsahrs[26]);     //俯仰角
        AHRSData_Packet.Heading=DATA_Trans(Fd_rsahrs[27],Fd_rsahrs[28],Fd_rsahrs[29],Fd_rsahrs[30]);	 //偏航角

        AHRSData_Packet.Qw=DATA_Trans(Fd_rsahrs[31],Fd_rsahrs[32],Fd_rsahrs[33],Fd_rsahrs[34]);  //四元数
        AHRSData_Packet.Qx=DATA_Trans(Fd_rsahrs[35],Fd_rsahrs[36],Fd_rsahrs[37],Fd_rsahrs[38]);
        AHRSData_Packet.Qy=DATA_Trans(Fd_rsahrs[39],Fd_rsahrs[40],Fd_rsahrs[41],Fd_rsahrs[42]);
        AHRSData_Packet.Qz=DATA_Trans(Fd_rsahrs[43],Fd_rsahrs[44],Fd_rsahrs[45],Fd_rsahrs[46]);
        AHRSData_Packet.Timestamp=timestamp(Fd_rsahrs[47],Fd_rsahrs[48],Fd_rsahrs[49],Fd_rsahrs[50]);   //时间戳
        //AHRSData2PC();
        }
    rs_ahrstype=0;
    }

    if(rs_imutype==1)
    {
        if(Fd_rsimu[1]==TYPE_IMU&&Fd_rsimu[2]==IMU_LEN)
        {
        IMUData_Packet.gyroscope_x=DATA_Trans(Fd_rsimu[7],Fd_rsimu[8],Fd_rsimu[9],Fd_rsimu[10]);  //角速度
        IMUData_Packet.gyroscope_y=DATA_Trans(Fd_rsimu[11],Fd_rsimu[12],Fd_rsimu[13],Fd_rsimu[14]);
        IMUData_Packet.gyroscope_z=DATA_Trans(Fd_rsimu[15],Fd_rsimu[16],Fd_rsimu[17],Fd_rsimu[18]);

        IMUData_Packet.accelerometer_x=DATA_Trans(Fd_rsimu[19],Fd_rsimu[20],Fd_rsimu[21],Fd_rsimu[22]);  //线加速度
        IMUData_Packet.accelerometer_y=DATA_Trans(Fd_rsimu[23],Fd_rsimu[24],Fd_rsimu[25],Fd_rsimu[26]);
        IMUData_Packet.accelerometer_z=DATA_Trans(Fd_rsimu[27],Fd_rsimu[28],Fd_rsimu[29],Fd_rsimu[30]);

        IMUData_Packet.magnetometer_x=DATA_Trans(Fd_rsimu[31],Fd_rsimu[32],Fd_rsimu[33],Fd_rsimu[34]);  //磁力计数据
        IMUData_Packet.magnetometer_y=DATA_Trans(Fd_rsimu[35],Fd_rsimu[36],Fd_rsimu[37],Fd_rsimu[38]);
        IMUData_Packet.magnetometer_z=DATA_Trans(Fd_rsimu[39],Fd_rsimu[40],Fd_rsimu[41],Fd_rsimu[42]);

        IMUData_Packet.Timestamp=timestamp(Fd_rsimu[55],Fd_rsimu[56],Fd_rsimu[57],Fd_rsimu[58]);   //时间戳
        //IMUData2PC();
        }
        rs_imutype=0;
    }
    return 0;
}

void* Thread_USB_MEMS(void*)//not use now
{

    int i;
    static uint8_t state_usb=0, rx_cnt_usb=0;
    static uint8_t _data_len2_usb = 0, _data_cnt2_usb = 0;
    int ret=0;
    int rx_length=42;
    uint8_t data_usb = 0;
    int iResult = -1;
    int fd = -1,iCommPort,iBaudRate,iDataSize,iStopBit;
    char cParity;
    int iLen;

    serial::Serial m_serial;
    m_serial.setPort( "/dev/ttyUSB0");
    m_serial.setBaudrate(921600);
    m_serial.setBytesize(serial::eightbits);
    m_serial.setParity(serial::parity_none);
    m_serial.setStopbits(serial::stopbits_one);
    m_serial.open();
    while(m_serial.isOpen()==0)
    {
        m_serial.open();
        usleep(100000);
    }
//------------------------------------------memery-------------------------------------
    static float timer_spi1 = 0,timer_spi2=0;
    static int mem_init_cnt=0;
    float sys_dt = 0;
    int memory_update=0;
    int flag = 0;
    int link_cnt=0;
    char buf_mem[MEM_SIZE]={1,2};
    Cycle_Time_Init();

    fd_set  rset;
    int  rv = -1 ;
    int nread=0;
    struct timeval timeout;
    timeout.tv_sec=0;
    timeout.tv_usec=1000;

    static char Count=0;
    static char rs_count=0;
    static char last_rsnum=0;
    static char rsimu_flag=0;
    static char rsacc_flag=0;
    int att_cal=0,gyro_cal=0;
    //read bias
    spi_rx.att_usb_bias[0]=config_hardware["mems_param"]["att_bias"][0].as<float>();//
    spi_rx.att_usb_bias[1]=config_hardware["mems_param"]["att_bias"][1].as<float>();//
    spi_rx.att_usb_bias[2]=config_hardware["mems_param"]["att_bias"][2].as<float>();//
    spi_rx.att_rate_usb_bias[0]=config_hardware["mems_param"]["gyro_bias"][0].as<float>();//
    spi_rx.att_rate_usb_bias[1]=config_hardware["mems_param"]["gyro_bias"][1].as<float>();//
    spi_rx.att_rate_usb_bias[2]=config_hardware["mems_param"]["gyro_bias"][2].as<float>();//

    while (1)
    {
        sys_dt = Get_Cycle_T(20);
        mems_usb_loss_cnt += sys_dt;
        if (mems_usb_loss_cnt > 1.5&& mems_usb_connect==1)
        {
            mems_usb_loss_cnt = 0;
            mems_usb_connect = 0;
            printf("Hardware::Hardware USB-MEMS Loss!!!\n");
        }

        nread = m_serial.available();
        if (nread>0){
            //printf(" \n read_cefore=%d\n",nread);
            m_serial.read(buff, nread);
        }

        for (int i = 0; i < nread; i++)
        {
            //printf("%02x ",buff[i]);
            Fd_data[Count]=buff[i];  //串口数据填入数组
            if(((last_rsnum==FRAME_END)&&(buff[i] == FRAME_HEAD))||Count>0)
                {
                rs_count=1;
                Count++;

                mems_usb_loss_cnt=0;
                if(!mems_usb_connect){
                 printf("Hardware::Hardware USB-MEMS Connect-In!!!\n");
                 mems_usb_connect=1;
                }

                if((Fd_data[1]==TYPE_IMU)&&(Fd_data[2]==IMU_LEN))
                   { rsimu_flag=1;rsacc_flag=0;}
                else if((Fd_data[1]==TYPE_AHRS)&&(Fd_data[2]==AHRS_LEN))
                   { rsacc_flag=1;rsimu_flag=0;}
                }
                else
                    Count=0;
                last_rsnum=buff[i];

            if(rsimu_flag==1 && Count==IMU_RS) //将本帧数据保存至Fd_rsimu数组中
            {
                Count=0;
                rsimu_flag=0;
                rs_imutype=1;
                if(Fd_data[IMU_RS-1]==FRAME_END) //帧尾校验
                memcpy(Fd_rsimu, Fd_data, sizeof(Fd_data));
            }
            else if(rsacc_flag==1 && Count==AHRS_RS) //
            {
                Count=0;
                rsacc_flag=0;
                rs_ahrstype=1;
                if(Fd_data[AHRS_RS-1]==FRAME_END)
                memcpy(Fd_rsahrs, Fd_data, sizeof(Fd_data));
            }
        }

      TTL_Hex2Dec();

      //cal imu
      if(att_cal==0&&(mems.Acc_CALIBRATE))
          att_cal=1;
     // printf("%d %d\n",att_cal,mems.Acc_CALIBRATE);
      if(att_cal==1){
        att_cal=2;
        spi_rx.att_usb_bias[0]=spi_rx.att_usb[0];
        spi_rx.att_usb_bias[1]=spi_rx.att_usb[1];
        config_hardware["mems_param"]["att_bias"][0]=spi_rx.att_usb_bias[0];//
        config_hardware["mems_param"]["att_bias"][1]=spi_rx.att_usb_bias[1];//
        config_hardware["mems_param"]["att_bias"][2]=spi_rx.att_usb_bias[2];//
        std::ofstream fout_hard("/home/odroid/Corgi/Param/param_hardware.yaml");
        fout_hard<<config_hardware;
        fout_hard.close();
      }
      if(att_cal==2&&mems.Acc_CALIBRATE==0&&mems.Gyro_CALIBRATE==0){
          printf("Hardware::IMU Mems Att bias is [%f] [%f] [%f]\n",spi_rx.att_usb_bias[0],spi_rx.att_usb_bias[1],spi_rx.att_usb_bias[2]);
          att_cal=0;
      }

      if(gyro_cal==0&&(mems.Gyro_CALIBRATE))
          gyro_cal=1;

      if(gyro_cal==1){
        gyro_cal=2;
        spi_rx.att_rate_usb_bias[0]=spi_rx.att_rate_usb[0];
        spi_rx.att_rate_usb_bias[1]=spi_rx.att_rate_usb[1];
        spi_rx.att_rate_usb_bias[2]=spi_rx.att_rate_usb[2];
        config_hardware["mems_param"]["gyro_bias"][0]=spi_rx.att_rate_usb_bias[0];//
        config_hardware["mems_param"]["gyro_bias"][1]=spi_rx.att_rate_usb_bias[1];//
        config_hardware["mems_param"]["gyro_bias"][2]=spi_rx.att_rate_usb_bias[2];//
        std::ofstream fout_hard("/home/odroid/Corgi/Param/param_hardware.yaml");
        fout_hard<<config_hardware;
        fout_hard.close();
      }
      if(gyro_cal==2&&mems.Acc_CALIBRATE==0&&mems.Gyro_CALIBRATE==0){
          printf("Hardware::IMU Mems Gyro bias is [%f] [%f] [%f]\n",spi_rx.att_rate_usb_bias[0],spi_rx.att_rate_usb_bias[1],spi_rx.att_rate_usb_bias[2]);
          gyro_cal=0;
      }
      usleep(2*1000);
    }
    close(fd);

    return 0;
}


int main(int argc, char *argv[])
{
    int use_usb_imu=config_hardware["mems_param"]["imu_usb_enable"].as<float>();//
#if NO_THREAD&&!EN_MULTI_THREAD&&!USE_USB
    Thread_ALL();
#else
    pthread_t tida, tidb,tidc;
    pthread_mutex_init(&lock, NULL);
    #if EN_MULTI_THREAD&&!USE_USB
    pthread_create(&tida, NULL, Thread_Mem, NULL);
    pthread_create(&tida, NULL, Thread_SPI, NULL);
    if(use_usb_imu)
        pthread_create(&tidc, NULL, Thread_USB_MEMS, NULL);
    pthread_join(tida, NULL);
    pthread_join(tidb, NULL);
    if(use_usb_imu)
        pthread_join(tidc, NULL);
    #else
#if !USE_USB
    pthread_create(&tida, NULL, Thread_ALL, NULL);
    pthread_join(tida, NULL);
#else
    pthread_create(&tidb, NULL, Thread_Mem, NULL);
    pthread_create(&tidc, NULL, Thread_USB, NULL);
#endif
#if EN_MULTI_THREAD
    pthread_join(tidb, NULL);
    pthread_join(tidc, NULL);
#endif
    #endif
#endif
    return 1;
}
