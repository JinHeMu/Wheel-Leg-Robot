#include "thread_init.h"

/* 定义线程控制块和栈 */
static struct rt_thread INS_th;
static rt_uint8_t INS_stack[1024];  // 1KB栈

static struct rt_thread chassisL_th;
static rt_uint8_t chassisL_stack[1024];

static struct rt_thread chassisR_th;
static rt_uint8_t chassisR_stack[1024];

static struct rt_thread ps2_th;
static rt_uint8_t ps2_stack[1024];

static struct rt_thread observe_th;
static rt_uint8_t observe_stack[1024];

extern JOYSTICK_TypeDef my_joystick;

/* 线程入口函数 */
void INS_entry(void *parameter)
{
    INS_task();
}

void chassisL_entry(void *parameter)
{
    ChassisL_task();
}

void chassisR_entry(void *parameter)
{
    ChassisR_task();
}

void ps2_entry(void *parameter)
{
//    pstwo_task();

		while(1)
		
	{
		AX_PS2_ScanKey(&my_joystick);
		rt_kprintf("mode:%d,btn1:%d,btn2:%d,RL:%d,RU:%d,LL:%d,LR:%d\n",my_joystick.mode,my_joystick.btn1,my_joystick.btn2,my_joystick.RJoy_LR,my_joystick.RJoy_UD,my_joystick.LJoy_LR,my_joystick.LJoy_UD);
		rt_thread_mdelay(30);
		
	}
}

void observe_entry(void *parameter)
{
    Observe_task();
}

/* 静态线程初始化函数 */
void INS_init(void)
{
    rt_thread_init(&INS_th,
                  "INS_th",
                  INS_entry,
                  RT_NULL,
                  INS_stack,
                  sizeof(INS_stack),
                  11,  // 优先级
                  1);  // 时间片
    
    rt_thread_startup(&INS_th);
}

void chassisL_init(void)
{
    rt_thread_init(&chassisL_th,
                  "chassisL_th",
                  chassisL_entry,
                  RT_NULL,
                  chassisL_stack,
                  sizeof(chassisL_stack),
                  13,
                  1);
    
    rt_thread_startup(&chassisL_th);
}

void chassisR_init(void)
{
    rt_thread_init(&chassisR_th,
                  "chassisR_th",
                  chassisR_entry,
                  RT_NULL,
                  chassisR_stack,
                  sizeof(chassisR_stack),
                  13,
                  1);
    
    rt_thread_startup(&chassisR_th);
}

void ps2_init(void)
{
    rt_thread_init(&ps2_th,
                  "ps2_th",
                  ps2_entry,
                  RT_NULL,
                  ps2_stack,
                  sizeof(ps2_stack),
                  14,
                  1);
    
    rt_thread_startup(&ps2_th);
}

void observe_init(void)
{
    rt_thread_init(&observe_th,
                  "observe_th",
                  observe_entry,
                  RT_NULL,
                  observe_stack,
                  sizeof(observe_stack),
                  12,
                  1);
    
    rt_thread_startup(&observe_th);
}

/* 统一初始化函数 */
void thread_init(void)
{
//    INS_init();
//    chassisL_init();
//    chassisR_init();
//    observe_init();
    ps2_init();
}
MSH_CMD_EXPORT(thread_init, thread_init);
