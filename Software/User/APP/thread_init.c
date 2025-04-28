#include "thread_init.h"

rt_thread_t INS_th;
rt_thread_t chassisL_th;
rt_thread_t chassisR_th;
rt_thread_t ps2_th;
rt_thread_t observe_th;

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
	pstwo_task();
}

void observe_entry(void *parameter)
{
	Observe_task();
}

void INS_init(void)
{

	// 创建
	INS_th = rt_thread_create("INS_th", INS_entry, RT_NULL, 1024, 11, 1);

	// 启动
	if (RT_NULL != INS_th)
	{
		rt_thread_startup(INS_th);
	}
}

void chassisL_init(void)
{

	// 创建
	chassisL_th = rt_thread_create("chassisL_th", chassisL_entry, RT_NULL, 1024, 13, 1);

	// 启动
	if (RT_NULL != chassisL_th)
	{
		rt_thread_startup(chassisL_th);
	}
}
void chassisR_init(void)
{

	// 创建
	chassisR_th = rt_thread_create("chassisR_th", chassisR_entry, RT_NULL, 1024, 13, 1);

	// 启动
	if (RT_NULL != chassisR_th)
	{
		rt_thread_startup(chassisR_th);
	}
}

void ps2_init(void)
{

	// 创建
	ps2_th = rt_thread_create("ps2_th", ps2_entry, RT_NULL, 1024, 14, 1);

	// 启动
	if (RT_NULL != ps2_th)
	{
		rt_thread_startup(ps2_th);
	}
}

void observe_init(void)
{

	// 创建
	observe_th = rt_thread_create("observe_th", observe_entry, RT_NULL, 1024, 12, 1);

	// 启动
	if (RT_NULL != observe_th)
	{
		rt_thread_startup(observe_th);
	}
}

void thread_init(void)
{
	INS_init();
	chassisL_init();
	chassisR_init();
	observe_init();
	ps2_init();
}MSH_CMD_EXPORT(thread_init, thread_init);
