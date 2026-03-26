#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "Chassis.h"
#include "CANDrive.h"
#include "semphr.h"
#include "comm.h"
#include "dataFrame.h"
extern SemaphoreHandle_t Jy61_semaphore;
extern SemaphoreHandle_t remote_semaphore;
extern SemaphoreHandle_t Remote_semaphore;
extern JY61_Typedef JY61;
PackControl_t recv_pack;
Remote_Handle_t Remote_Control; //取出遥控器数据
TaskHandle_t Remote_Jy61_Task_Handle;
TaskHandle_t Remote_Handle;
//Chassis_t chassis;
uint8_t recv_buff[20] = {0};
PackControl_t recv_pack_remote;
uint8_t usart4_dma_buff[30];
uint8_t usart5_dma_buff[30];
ChassisMode chassis_mode = REMOTE;
float rocker_filter[4] = {0};

void Task_Init()
{

  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
	
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff, sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	xTaskCreate(Remote,
         "Remote",
          400,
          NULL,
          4,
          &Remote_Handle); 

	xTaskCreate(Hit_Task,
			 "Hit_Task",
				400,
				NULL,
				4,
				&Hit_Task_Handle); 
				
				xTaskCreate(Remote_Jy61,
			 "Remote_Jy61",
				400,
				NULL,
				4,
				&Hit_Task_Handle); 
}
static void Key_Parse(uint32_t key, hw_key_t *out)
{
    out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}
void Remote_Analysis()
{
    if(xSemaphoreTake(Remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      /* 1. 保存上一帧 */
      Remote_Control.Second = Remote_Control.First;
      /* 2. 解析当前按键 */
      Key_Parse(recv_pack.Key, &Remote_Control.First);

      Remote_Control.Ex = recv_pack.rocker[0] / 1847.0f *MAX_ROBOT_VEL;
      Remote_Control.Ey = recv_pack.rocker[1] / 1847.0f *MAX_ROBOT_VEL;
      Remote_Control.Eomega = recv_pack.rocker[2] / 1847.0f * MAX_ROBOT_OMEGA;
    }else {
      Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;

      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
    }
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
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    Rocker_Filter(&recv_pack);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(Remote_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;

 void Remote_Jy61(void *pvParameters)
  {
      TickType_t last_wake_time = xTaskGetTickCount();
      g_comm_handle = Comm_Init(&huart5);
      RemoteCommInit(NULL);
      register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
      for(;;)
      {
          if(xSemaphoreTake(Jy61_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
          {
              JY61_Receive(&JY61, usart5_dma_buff, sizeof(JY61));
          }
      }
  }
	 //电机驱动
VESC_INIT vesc_1 ={
	.steer.motor_id = 0x01,
	.steer.hcan = &hcan2,
	
	
};
VESC_INIT vesc_2 ={
	.steer.motor_id = 0x02,
	.steer.hcan = &hcan2,
	
	
};
VESC_INIT vesc_3 ={
	.steer.motor_id = 0x03,
	.steer.hcan = &hcan2,
	
	
};
float Vx =0;   //前后移动
float Vy =0;   //左右移动
float Wz =0;   //顺逆自转
volatile float v1 = 0.0f;
volatile float v2 = 0.0f;
volatile float v3 = 0.0f;
volatile float wheel_one = 0.0f;  //前左
volatile float wheel_two = 0.0f;  //前右
volatile float wheel_three=0.0f;  //后右
TaskHandle_t Remote_Handle;
void Remote(void *pvParameters)
{
	portTickType xLastWakeTime = xTaskGetTickCount();

	vesc_1.PID.Kp =2.5f;
	vesc_1.PID.Ki = 0.05f;
	vesc_1.PID.Kd = 20.0f;
	vesc_1.PID.limit = 10000.0f;
	vesc_1.PID.output_limit = 40.0f;
	vesc_2.PID.Kp =2.5f;
	vesc_2.PID.Ki = 0.05f;
	vesc_2.PID.Kd = 20.0f;
	vesc_2.PID.limit = 10000.0f;
	vesc_2.PID.output_limit = 40.0f;
	vesc_3.PID.Kp =2.5f;
	vesc_3.PID.Ki = 0.05f;
	vesc_3.PID.Kd = 20.0f;
	vesc_3.PID.limit = 100000.0f;
	vesc_3.PID.output_limit = 40.0f;
	vesc_1.dead_area = 5.0f;
	vesc_2.dead_area = 5.0f;
	vesc_3.dead_area = 5.0f;
    PID_EREOR_Init(&vesc_1.PID_ERROR,vesc_1.PID.Kp,vesc_1.PID.Ki,vesc_1.PID.Kd,vesc_1.PID.output_limit,vesc_1.PID.limit,vesc_1.dead_area,0.0f);
   	PID_EREOR_Init(&vesc_2.PID_ERROR,vesc_2.PID.Kp,vesc_2.PID.Ki,vesc_2.PID.Kd,vesc_2.PID.output_limit,vesc_2.PID.limit,vesc_2.dead_area,0.0f);
    PID_EREOR_Init(&vesc_3.PID_ERROR,vesc_3.PID.Kp,vesc_3.PID.Ki,vesc_3.PID.Kd,vesc_3.PID.output_limit,vesc_3.PID.limit,vesc_3.dead_area,0.0f);

	for(;;)
	{
    float roll = JY61.Angle.Roll;
    float pitch = JY61.Angle.Pitch;
		
		v1 = -Remote_Control.Ex*0.5f+ Remote_Control.Ey*(sqrt (3.0f)/2.0)+LENGTH * Remote_Control.Eomega;
		v2 = -Remote_Control.Ex*0.5f- Remote_Control.Ey*(sqrt (3.0f)/2.0)+LENGTH * Remote_Control.Eomega;
        v3 =  Remote_Control.Ex +LENGTH * Wz;

		wheel_one=  -(int16_t)((v1 / (2.0f * PI * WHEEL_RADIUS)) );
		wheel_two=   (int16_t)((v2 / (2.0f * PI * WHEEL_RADIUS)) );
		wheel_three=-(int16_t)((v3 / (2.0f * PI * WHEEL_RADIUS)) );
			
		PID_Control2((float)((float)vesc_1.steer.epm / 7.0f/(3.4f)), (wheel_one   ), &vesc_1.PID);
		PID_Control2((float)((float)vesc_2.steer.epm / 7.0f/(3.4f)), (wheel_two   ), &vesc_2.PID);
		PID_Control2((float)((float)vesc_3.steer.epm / 7.0f/(3.4f)), (wheel_three ), &vesc_3.PID);

		
	    VESC_SetCurrent(&vesc_1.steer, vesc_1.PID_ERROR.output);
        VESC_SetCurrent(&vesc_2.steer, vesc_2.PID_ERROR.output);
	    VESC_SetCurrent(&vesc_3.steer, vesc_3.PID_ERROR.output);

			Remote_Analysis();
			/* 单次触发 */
			if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Switch_Up))
			{
				chassis_mode = REMOTE;
			}
			if (KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Switch_Down))
			{
				chassis_mode = AUTO;
			}
      
      if(chassis_mode == REMOTE)
      {
//        chassis.exp_vel.x = Remote_Control.Ex;
//        chassis.exp_vel.y = Remote_Control.Ey;
//        chassis.exp_vel.z = Remote_Control.Eomega;
      }else if(chassis_mode == AUTO)
      {
        
      }
		vTaskDelayUntil(&xLastWakeTime,2);
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	VESC_ReceiveHandler(&vesc_1.steer, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&vesc_2.steer, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&vesc_3.steer, &hcan2, ID,Recv);
    }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
   		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        HAL_UART_DMAStop(huart);
        // 重置HAL状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        
        // 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
        uint32_t isrflags = READ_REG(huart->Instance->SR);
        
        // 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
        if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
        {
            // 对于ORE、NE、FE错误，需要先读SR再读DR
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        

        if (isrflags & USART_SR_PE)
        {
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
        }
        
    }
      Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff, 0);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
      __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}
