# Hyleg
## STM32 
### 初始化
1. IO初始化
2. CAN初始化
3. SPI初始化

### 线程调度
线程调度函数，设定不同周期的定时器完成不同的任务。
1. 1ms 完成CAN数据通信和USB上位机通信；
2. 2ms完成姿态解算；
3. 
### IMU采集
1. 加速度计采集
2. 陀螺仪采集
3. 磁力计采集

### 姿态解算


### SPI主控通讯

### CAN通讯
1. CAN口发送函数
```
    can_send(int motor_address,unsigned char* send_buf,int send_len)
```
### 电机函数
1. 电机使能
```
int motor_enable(int motor_address)
```
2. 电机失能
```
int motor_disable(int motor_address)
```
3. 电机控制
```
int pack_TX(int motor_address, float p_des, float v_des, float kp, float kd, float t_ff)
```

控制电机位置、速度、KP、KD、扭矩

4. 电机接收
```
struct Tmotor unpack_RX(unsigned char rx_buf[6])
```
接收电机返回信息，存入Tmotor
## odroid

