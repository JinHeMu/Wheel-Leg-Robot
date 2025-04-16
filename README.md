# 轮足机器人设计

## 介绍

本设计基于[达妙科技开源轮足](https://gitee.com/kit-miao/wheel-legged),使用4个DM4310关节和两个DM6015轮毂,一个DM-MC02开发板,并使用STM32CubeMX V6.10.0, STM32Cube FW_H7 V1,11,2,MDK-ARM V5.32作为开发环境,并移植RT-Thread Nano V3.1.3实时操作系统.

本项目仍存在RT-Thread Nano msh控制台无法输入上下方向键问题.原计划使用Clion + gcc作为编译环境,但是在移植RT-Thread Nano之后出现无法使用msh的问题,所以本项目先使用MDK-ARM环境.

## 软件架构





