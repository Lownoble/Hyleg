# Hyleg
## STM32 
### 初始化
### 线程调度
### IMU采集
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
3. 电机控制代码
```
    int pack_TX(int motor_address, float p_des, float v_des, float kp, float kd, float t_ff)
```
    控制电机位置、速度、KP、KD、扭矩

## odroid

