# 轮足机器人设计

## 介绍

本设计基于[达妙科技开源轮足](https://gitee.com/kit-miao/wheel-legged),用于毕业设计。使用4个DM4310关节和两个DM6215轮毂,一个DM-MC02开发板,并使用STM32CubeMX V6.14.0, STM32Cube FW_H7 V1.12.1,MDK-ARM V5.32作为开发环境,并移植RT-Thread Nano V4.1.1实时操作系统。

本项目使用中断接受终端信息,已解决RT-Thread Nano msh控制台无法输入上下方向键问题（使用中断接受）。原计划使用Clion + gcc作为编译环境,但是在移植RT-Thread Nano之后出现问题,所以本项目先使用MDK-ARM环境。

