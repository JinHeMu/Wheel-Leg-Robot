# 轮足机器人设计

## 介绍

本设计基于[达妙科技开源轮足](https://gitee.com/kit-miao/wheel-legged),使用4个DM4310关节和两个DM6015轮毂,一个DM-MC02开发板,并使用STM32CubeMX V6.14.0, STM32Cube FW_H7 V1.12.1,MDK-ARM V5.32作为开发环境,并移植RT-Thread Nano V4.1.1实时操作系统.

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

