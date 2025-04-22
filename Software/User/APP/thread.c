#include "thread.h"

rt_thread_t INS_th;


void INS_entry(void *parameter)
{
	while (1)
	{
		INS_task();
	}
}

void INS_init(void)
{


	// ´´½¨
	INS_th = rt_thread_create("INS", INS_entry, RT_NULL, 1024, 11, 1);

	// Æô¶¯
	if (RT_NULL != INS_th)
	{
		rt_thread_startup(INS_th);
	}
}MSH_CMD_EXPORT(INS_init,INS init);

