#include "hit_ball.h"
GPIO_PinState GPIOA8_State = GPIO_PIN_SET;
GPIO_PinState GPIOC9_State = GPIO_PIN_SET;
int take = 1;
TaskHandle_t Hit_Task_Handle;
GPIO_PinState key1, key2, key3;
uint8_t hit_ball_trigger = 0;
uint8_t flag = 0;
void Hit_Task(void *pvParameters)
{
TickType_t Last_wake_time = xTaskGetTickCount();
for(;;)
	{
    key1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
		key2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		key3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		if(key1 == GPIO_PIN_SET || key2 == GPIO_PIN_SET || key3 == GPIO_PIN_SET)
			{
			flag = 1;	//这里做自动发球，手动发球方式暂存（chassis.c）
			if(flag == 1)
			  {
				hit_ball_trigger = 1;
	       }
	    }
		 if(hit_ball_trigger == 1)
			{
//此为测试使用
			  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIOA8_State);
			  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIOC9_State);
//			    //启动电磁阀门进行击球				
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
//					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
//					vTaskDelay(500);
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
//					hit_ball_trigger = 0;
//					flag = 2;
			}


	vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
  }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{


    
}
