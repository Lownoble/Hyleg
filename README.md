# Hyleg
## STM32 

### TodoList
- [x] 初始化
- [x] 下位机电机限位（速度、位置）
- [x] 脚尖触地反馈
- [x] 机身位置速度状态估计
- [ ] 自适应负重


### 初始化
1. IO初始化
2. CAN初始化
3. SPI初始化
4. 电机初始化
 - 设置电机ID
 - 等待询问设置电机零点（Z/N）
 - 等待开始指令（Y/N）

----
### 线程调度
线程调度函数，设定不同周期的定时器完成不同的任务。
1. 1ms 完成CAN数据通信和USB上位机通信；
2. 2ms完成姿态解算；


----
### IMU采集
1. 加速度计采集
2. 陀螺仪采集
3. 磁力计采集

+ 问题：
  - 输出姿态角度不发生变化。
  -

原因：STM32与MPU9250未完成通信，无法收到SPI的消息。
    MPU9250读取数据均为0，可能是初始化未完成。

----
### 姿态解算

----
### SPI主控通讯


----
### CAN通讯
1. CAN口发送函数
```
can_send(int motor_address,unsigned char* send_buf,int send_len)
```
----
### 电机函数

电机限位：
![简图](https://github.com/Lownoble/Hyleg/blob/master/images/plot.png)
- 1.竖直为0位，[-1.25,1.37] 45°[-2.035,0.585]
- 2.垂直为0位，[-0.67,1.14] 45°[]
----

1. 电机使能
```
int motor_enable(int motor_address)
```
1. 电机失能
```
int motor_disable(int motor_address)
```
1. 电机控制
```
int pack_TX(int motor_address, float p_des, float v_des, float kp, float kd, float t_ff)
```

控制电机位置、速度、KP、KD、扭矩

4. 电机接收
```
struct Tmotor unpack_RX(unsigned char rx_buf[6])
```
接收电机返回信息，存入Tmotor

------
## odroid

### 运动控制线程（500Hz）



### SPI通信线程（200Hz）



### 串口通信线程（100Hz）


