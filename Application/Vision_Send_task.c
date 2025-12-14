#include "Vison_Send_task.h"

void Task_Vision_Send(void const *argu)
{
    uint32_t wake_time = osKernelSysTick();

    for (;;)
    {
        taskENTER_CRITICAL();

        vision_output_data();

        taskEXIT_CRITICAL();

     osDelayUntil(&wake_time, 3);
    }
}
