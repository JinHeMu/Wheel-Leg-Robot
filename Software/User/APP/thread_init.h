#ifndef __THREAD_INIT_H
#define __THREAD_INIT_H

#include <rtthread.h>
#include "INS_task.h"
#include "chassisL_task.h"
#include "chassisR_task.h"
#include "observe_task.h"
#include "ps2_task.h"


void thread_init(void);

#endif // !__THREAD_H
