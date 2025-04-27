# 轮足机器人设计

## 介绍

本设计基于[达妙科技开源轮足](https://gitee.com/kit-miao/wheel-legged),使用4个DM4310关节和两个DM6215轮毂,一个DM-MC02开发板,并使用STM32CubeMX V6.14.0, STM32Cube FW_H7 V1.12.1,MDK-ARM V5.32作为开发环境,并移植RT-Thread Nano V4.1.1实时操作系统.

本项目使用中断接受终端信息,已解决RT-Thread Nano msh控制台无法输入上下方向键问题.原计划使用Clion + gcc作为编译环境,但是在移植RT-Thread Nano之后出现问题,所以本项目先使用MDK-ARM环境.

## 软件架构

### Bsp

#### bsp_dwt

Data Watchpoint and Trace,用于系统调试及跟踪,一种高精度定时器,计算获取时间差.

#### bsp_pwm

输出pwm占空比.

#### can_bsp

can通讯配置初始化,发送和接受can数据(中断),同时注意不同HAL库版本导致`FDCAN_DLC_BYTES_7`定义不同.

### Devices

#### dm4310_drv

关节和轮毂电机can反馈帧解算和can控制帧发送,使能和失能电机,mit,位置速度,速度控制.

#### BMI088driver

读取六轴陀螺仪加速度和速度信息,需手动修改零漂信息.

### Algorithm

#### mahony

欧拉角: Yaw,Pitch,Roll($\alpha, \beta, \gamma$ ).

旋转矩阵:

$$
\mathbf{R}_b^e = 
\begin{bmatrix}
1 - 2(q_2^2 + q_3^2) & 2(q_1q_2 - q_0q_3)     & 2(q_1q_3 + q_0q_2) \\
2(q_1q_2 + q_0q_3)   & 1 - 2(q_1^2 + q_3^2)   & 2(q_2q_3 - q_0q_1) \\
2(q_1q_3 - q_0q_2)   & 2(q_2q_3 + q_0q_1)     & 1 - 2(q_1^2 + q_2^2)
\end{bmatrix}
$$

$$
\mathbf{R}_b^e = \mathbf{R}_z(\psi) \mathbf{R}_y(\theta) \mathbf{R}_x(\phi) =
\begin{bmatrix}
\cos\psi \cos\theta & \cos\psi \sin\theta \sin\phi - \sin\psi \cos\phi & \cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi \\
\sin\psi \cos\theta & \sin\psi \sin\theta \sin\phi + \cos\psi \cos\phi & \sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi \\
-\sin\theta         & \cos\theta \sin\phi                              & \cos\theta \cos\phi
\end{bmatrix}
$$

$$
\begin{align*}
\alpha &= \arctan 2(R_{32}, R_{33}) \\
\beta &= -\arcsin(R_{31}) \\
\gamma &= \arctan 2(R_{21}, R_{11})
\end{align*}
$$

mahony滤波:用真实加速度修正积分的误差,计算输出欧拉角

### APP

#### INS_task

mahony方法获取机体姿态，同时获取机体在绝对坐标系下的运动加速度

chassisR_task

电机初始化使能,vmc初始化,腿长pid初始化
