#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "Chassis.h"
#include "CANDrive.h"
#include "semphr.h"
#include "comm.h"
#include "dataFrame.h"
extern SemaphoreHandle_t remote_semaphore;
uint8_t recv_buff[20] = {0};
PackControl_t recv_pack_remote;
TaskHandle_t Move_Task_Handle;

ChassisMode chassis_mode = REMOTE;
float rocker_filter[4] = {0};
void Task_Init()
{
	//Ò£¿ØÆ÷
//	    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//	    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
	
__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff, sizeof(usart5_buff));
__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	xTaskCreate(Remote,
         "Remote",
          400,
          NULL,
          4,
          &Remote_Handle); 
	
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
void Rocker_Filter(PackControl_t *data)
{
    float alpha = 0.6f;

    for(int i = 0; i < 4; i++)
    {
        rocker_filter[i] = alpha * data->rocker[i] +
                          (1.0f - alpha) * rocker_filter[i];

        data->rocker[i] = rocker_filter[i];
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_data, recv_buff, sizeof(recv_data));
    Rocker_Filter(&recv_data);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;
void Move_Task(void *pvParameters)
{
	  TickType_t last_wake_time = xTaskGetTickCount();
    g_comm_handle = Comm_Init(&huart5);
    RemoteCommInit(NULL);
    register_comm_recv_cb(recv_cb, 0x01, &recv_data);
	for(;;)
	{
//			if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
//			{
//					memcpy(&recv_data, usart5_buff, sizeof(recv_data));
					Updatakey(&Remote_Control);
					Remote_Control.Ex =-recv_data.rocker[1];
					Remote_Control.Ey = recv_data.rocker[0];
					Remote_Control.Eomega = recv_data.rocker[2];
					Remote_Control.mode = recv_data.rocker[3];
					Remote_Control.Key_Control = (hw_key_t*)&recv_data.Key;
//			}else{
//					recv_pack.Ex = 0;
//					recv_pack.Ey = 0;
//					recv_pack.Eomega = 0;
//					recv_pack.mode = 0;
//					//°´¼ü×´Ì¬ÇåÁã
//					memset(&recv_data.Key, 0, sizeof(hw_key_t));
//					recv_pack.Key_Control = &recv_data.Key;
//			}
//	}
}
	}

void Updatakey(Remote_Handle_t * xx) { //Ò£¿ØÆ÷Êý¾Ý¸üÐÂ
    xx->Second = xx->First;
    xx->First = *xx->Key_Control;
}

