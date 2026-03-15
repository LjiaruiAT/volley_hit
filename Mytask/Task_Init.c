#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "Chassis.h"
#include "CANDrive.h"
#include "semphr.h"
extern SemaphoreHandle_t remote_semaphore;

TaskHandle_t Move_Task_Handle;

ChassisMode chassis_mode = REMOTE;

void Task_Init()
{
	//Ò£¿ØÆ÷
//	    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//	    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
	
	    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff, sizeof(usart5_buff));
//	xTaskCreate(Remote,
//         "Remote",
//          400,
//          NULL,
//          4,
//          &Remote_Handle); 
//	
//	xTaskCreate(Move_Task,
//				"Move_Task",
//				200, NULL,
//				5,
//				&Move_Task_Handle);//Ò£¿ØÆ÷ÈÎÎñ

	xTaskCreate(Hit_Task,
			 "Hit_Task",
				400,
				NULL,
				4,
				&Hit_Task_Handle); 
}


void Move_Task(void *pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();

	for(;;)
	{
			if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
			{
					memcpy(&recv_data, usart5_buff, sizeof(recv_data));
					Updatakey(&recv_pack);
					recv_pack.Ex =-recv_data.rocker[1];
					recv_pack.Ey = recv_data.rocker[0];
					recv_pack.Eomega = recv_data.rocker[2];
					recv_pack.mode = recv_data.rocker[3];
					recv_pack.Key_Control = &recv_data.Key;
			}else{
					recv_pack.Ex = 0;
					recv_pack.Ey = 0;
					recv_pack.Eomega = 0;
					recv_pack.mode = 0;
					//°´¼ü×´Ì¬ÇåÁã
					memset(&recv_data.Key, 0, sizeof(hw_key_t));
					recv_pack.Key_Control = &recv_data.Key;
			}
	}
}

void Updatakey(Remote_Handle_t * xx) { //Ò£¿ØÆ÷Êý¾Ý¸üÐÂ
    xx->Second = xx->First;
    xx->First = *xx->Key_Control;
}

