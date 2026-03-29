#include "Task_Init.h"
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
				
 
}
//-------------------------Remote_Analysis--------------------------------
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


	//-------------------------------Remote_Move----------------------------------------
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
void Remote(void *pvParameters)
{
    portTickType xLastWakeTime = xTaskGetTickCount();
      g_comm_handle = Comm_Init(&huart5);
      RemoteCommInit(NULL);
      register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
    vesc_1.PID.Kp =0.0f;
   	vesc_1.PID.Ki = 0.0f;
	vesc_1.PID.Kd = 0.0f;
    vesc_1.PID.limit = 10000.0f;
	vesc_1.PID.output_limit = 40.0f;
    vesc_2.PID.Kp =0.0f;
	vesc_2.PID.Ki = 0.0f;
  	vesc_2.PID.Kd = 0.0f;
    vesc_2.PID.limit = 10000.0f;
  	vesc_2.PID.output_limit = 40.0f;
    vesc_3.PID.Kp =0.0f;
	vesc_3.PID.Ki = 0.0f;
  	vesc_3.PID.Kd = 0.0f;
    vesc_3.PID.limit = 100000.0f;
	vesc_3.PID.output_limit = 40.0f;
    for(;;)
    {
        v1 = -Remote_Control.Ex*0.5f + Remote_Control.Ey*(sqrt(3.0f)/2.0) + LENGTH * Remote_Control.Eomega;
        v2 = -Remote_Control.Ex*0.5f - Remote_Control.Ey*(sqrt(3.0f)/2.0) + LENGTH * Remote_Control.Eomega;
        v3 =  Remote_Control.Ex + LENGTH * Wz;

        wheel_one   = -(int16_t)(v1 / (2.0f * PI * WHEEL_RADIUS));
        wheel_two   =  (int16_t)(v2 / (2.0f * PI * WHEEL_RADIUS));
        wheel_three = -(int16_t)(v3 / (2.0f * PI * WHEEL_RADIUS));

        float wheel1_actual = vesc_1.steer.epm / 7.0f / 3.4f;
        float wheel2_actual = vesc_2.steer.epm / 7.0f / 3.4f;
        float wheel3_actual = vesc_3.steer.epm / 7.0f / 3.4f;

        float roll  = JY61.Angle.Roll;//foreward
        float pitch = JY61.Angle.Pitch;//left_right
        float yaw_rate = JY61.AngularVelocity.Z;//w
        float accX = JY61.Acceleration.X;//a
        float accY = JY61.Acceleration.Y;

        bool slip1 = (fabs(wheel_one - wheel1_actual) > 0.1f * fabs(wheel_one)) ||
                     (fabs(roll) > 3.0f) || (fabs(pitch) > 3.0f) || (fabs(yaw_rate) > 3.0f);
        bool slip2 = (fabs(wheel_two - wheel2_actual) > 0.1f * fabs(wheel_two)) ||
                     (fabs(roll) > 3.0f) || (fabs(pitch) > 3.0f) || (fabs(yaw_rate) > 3.0f);
        bool slip3 = (fabs(wheel_three - wheel3_actual) > 0.1f * fabs(wheel_three)) ||
                     (fabs(roll) > 3.0f) || (fabs(pitch) > 3.0f) || (fabs(yaw_rate) > 3.0f);

        PID_Control2(wheel1_actual, wheel_one, &vesc_1.PID);
        PID_Control2(wheel2_actual, wheel_two, &vesc_2.PID);
        PID_Control2(wheel3_actual, wheel_three, &vesc_3.PID);

        if(slip1) vesc_1.PID.pid_out *= 0.5f;
        if(slip2) vesc_2.PID.pid_out *= 0.5f;
        if(slip3) vesc_3.PID.pid_out *= 0.5f;

        VESC_SetCurrent(&vesc_1.steer, vesc_1.PID.pid_out);
        VESC_SetCurrent(&vesc_2.steer, vesc_2.PID.pid_out);
        VESC_SetCurrent(&vesc_3.steer, vesc_3.PID.pid_out);

        Remote_Analysis();

        if(KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Switch_Up))
            chassis_mode = REMOTE;
        if(KEY_RISING_EDGE(Remote_Control.First, Remote_Control.Second, Left_Switch_Down))
            chassis_mode = AUTO;

        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}

//-----------------------------Hit_Task------------------------------------------
void Hit_Task(void *pvParameters)
{
TickType_t Last_wake_time = xTaskGetTickCount();
for(;;)
	{	
		feel_1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    feel_2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
		feel_3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		feel_4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);

		if(feel_1 == GPIO_PIN_SET || feel_2 == GPIO_PIN_SET || feel_3 == GPIO_PIN_SET || feel_4 == GPIO_PIN_SET)
		{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
					vTaskDelay(1000);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
			}


	vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
  }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{  
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	VESC_ReceiveHandler(&vesc_1.steer, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&vesc_2.steer, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&vesc_3.steer, &hcan2, ID,Recv);
    }


