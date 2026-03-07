#include "hit_ball.h"

TaskHandle_t Hit_Task_Handle;

void Hit_Task(void *pvParameters)
{

TickType_t Last_wake_time = xTaskGetTickCount();
for(;;)
	{


	vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
  }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{


    
}
