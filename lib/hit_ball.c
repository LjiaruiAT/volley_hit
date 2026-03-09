#include "hit_ball.h"
int take = 1;
TaskHandle_t Hit_Task_Handle;

void Hit_Task(void *pvParameters)
{
cylinder_Init();
TickType_t Last_wake_time = xTaskGetTickCount();
for(;;)
	{
     if(take == 0)
		{
			  cylinder_down();
			
			
		}
       if(take == 1)
{
	cylinder_up();
}


	vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
  }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{


    
}
