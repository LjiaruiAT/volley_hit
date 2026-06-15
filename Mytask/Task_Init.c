#include "Task_Init.h"
void Remote_Analysis_Task(void *pvParameters);
TaskHandle_t Remote_Analysis_Handle;

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
						xTaskCreate(Remote_Analysis_Task, "Remote_Analysis_Task", 400, NULL, 4, &Remote_Analysis_Handle);

xTaskCreate(Hit_Task,
		 "Hit_Task",
			400,
			NULL,
			4,
			&Hit_Task_Handle); 
				
 xTaskCreate(Remote_JY61,
            "Remote_JY61",
            400,
            NULL,
            4,
            &Remote_JY61_Handle);
}
 static float lowpass_filter(float new_sample, float *prev_filter, float alpha)
  {
      *prev_filter = (1.0f - alpha) * (*prev_filter) + alpha * new_sample;
      return *prev_filter;
  }

  PID2 JY61_adjust = {
      .Kp           = 0.8f,     
      .Ki           = 0.0008f,  
      .Kd           = 0.35f,    
      .limit        = 8000.0f,  
      .output_limit = 60.0f,   
  };



  /* --------------------- 全局变量 --------------------- */
  volatile float Wz_correction = 0.0f;   // 陀螺仪校正量（给 Remote 任务使用）
  volatile float gyro_slip_val  = 0.0f;  // 当前滑移量（调试用）
  volatile uint8_t slip_flag    = 0;     // 打滑标志 (0=正常, 1=打滑中)

  int16_t motorCurrentBuf_3[4] = {0};
  float wheel1_actual = 0;
  float wheel2_actual = 0;
  float wheel3_actual = 0;
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
			
      /* 2. 解枝当剝按键 */
      Key_Parse(recv_pack.Key, &Remote_Control.First);

      Remote_Control.Ex = - recv_pack.rocker[0] / 1647.0f *MAX_ROBOT_VEL;
      Remote_Control.Ey = recv_pack.rocker[1] / 1647.0f *MAX_ROBOT_VEL;
      Remote_Control.Eomega = recv_pack.rocker[2] / 1647.0f * MAX_ROBOT_OMEGA;
    }else {
      Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;

      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    xSemaphoreGive(Remote_semaphore);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;


	//-------------------------------Remote_Move----------------------------------------

Motor3508Ex_t steer_1 = {
	.ID =0x201,
	.hcan = &hcan2,
};
Motor3508Ex_t steer_2 = {
	.ID =0x202,
	.hcan = &hcan2,
};
Motor3508Ex_t steer_3 = {
	.ID =0x203,
	.hcan = &hcan2,
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
		// 針置HAL状思
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然坎清除错误标志 - 按照STM32F4坂考手册覝求的顺庝
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺庝处睆坄秝错误标志，必须先读SR冝读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE〝NE〝FE错误，需覝先读SR冝读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读坖会清除ORE〝NE〝FE        

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
float RampAccel(float target, float current, float accel_step)
{
    if(target * current < 0)
    {
        return 0;
    }

    float diff = target - current;

    if(fabsf(target) > fabsf(current))
    {
        if(diff > accel_step)
            diff = accel_step;

        if(diff < -accel_step)
            diff = -accel_step;

        return current + diff;
    }

    return target;
}

void Remote(void *pvParameters)
{
    portTickType xLastWakeTime = xTaskGetTickCount();

    g_comm_handle = Comm_Init(&huart5);
    RemoteCommInit(NULL);
    register_comm_recv_cb(recv_cb, 0x01, &recv_pack);

    // ------------------- PID 初始化 -------------------
    steer_1.vel_pid.Kp = 22.0f; steer_1.vel_pid.Ki = 2.3f; steer_1.vel_pid.Kd = 1.0f;
    steer_1.vel_pid.limit = 16384.0f; steer_1.vel_pid.output_limit = 10000.0f;

    steer_2.vel_pid = steer_1.vel_pid;
    steer_3.vel_pid = steer_1.vel_pid;

    static float Ex_ref = 0;

    for(;;)
    {
			
        Ex_ref = RampAccel(Remote_Control.Ex, Ex_ref, 0.005f);

        float v1 = +Ex_ref*0.5f*MAX_VELOCITY
                   - Remote_Control.Ey*(sqrt(3.0f)/2.0f)*MAX_VELOCITY
                   - LENGTH * Remote_Control.Eomega*MAX_OMEGA;

        float v2 = -Ex_ref*0.5f*MAX_VELOCITY
                   - Remote_Control.Ey*(sqrt(3.0f)/2.0f)*MAX_VELOCITY
                   + LENGTH * Remote_Control.Eomega*MAX_OMEGA;

        float v3 = -Remote_Control.Eomega*MAX_OMEGA*LENGTH
                   - Ex_ref*(sqrt(3.0f)/2.0f)*MAX_VELOCITY;

        float wheel1_cmd = -(v1 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
        float wheel2_cmd =  (v2 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
        float wheel3_cmd = -(v3 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
if (fabsf(Wz_correction) > 0.01f)
          {
              
            float slip_rad    = Wz_correction * PI / 180.0f;                          // °/s → rad/s
                float slip_linear = slip_rad * LENGTH;                                    // rad/s → m/s
                float slip_rpm    = (slip_linear / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;   // m/s 
            
              float total_grip = WHEEL1_GRIP_RATIO + WHEEL2_GRIP_RATIO + WHEEL3_GRIP_RATIO;
              float w1 = WHEEL1_GRIP_RATIO / total_grip;  // ≈ 0.392
              float w2 = WHEEL2_GRIP_RATIO / total_grip;  // ≈ 0.392
              float w3 = WHEEL3_GRIP_RATIO / total_grip;  // ≈ 0.216

                wheel1_cmd += slip_rpm * w1;   
                wheel2_cmd -= slip_rpm * w2;  
                wheel3_cmd += slip_rpm * w3;   
          }
          
        wheel1_actual = (float)steer_1.motor.Speed;
        wheel2_actual = (float)steer_2.motor.Speed;
        wheel3_actual = (float)steer_3.motor.Speed;

        PID_Control2(wheel1_actual, wheel1_cmd, &steer_1.vel_pid);
        PID_Control2(wheel2_actual, wheel2_cmd, &steer_2.vel_pid);
        PID_Control2(wheel3_actual, wheel3_cmd, &steer_3.vel_pid);

        motorCurrentBuf_3[0] = (int16_t)(steer_1.vel_pid.pid_out);
        motorCurrentBuf_3[1] = (int16_t)(steer_2.vel_pid.pid_out);
        motorCurrentBuf_3[2] = (int16_t)(steer_3.vel_pid.pid_out);

        MotorSend(&hcan2, 0x200, motorCurrentBuf_3);

        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
void Remote_Analysis_Task(void *pvParameters)
{
TickType_t last_wake_time = xTaskGetTickCount();
while(1)
    {
        Remote_Analysis();

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(2));
    }
}
//-----------------------------Hit_Task------------------------------------------
uint8_t flag_out = 0;
static int test = 0;
uint8_t feel_1 = 0;
uint8_t feel_2 = 0;
uint8_t feel_3 = 0;
uint8_t feel_4 = 0;
typedef enum {
    BALL_IDLE = 0,
	  BALL_R_UP,
    BALL_RESET
} BallState_t;
void Hit_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    static TickType_t hit_end_time = 0;
    static uint8_t hitting = 0;

    for(;;)
    {	
        uint8_t feel_1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
        uint8_t feel_2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
        uint8_t feel_3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
        uint8_t feel_4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
        if(test == 0)
        {if  (feel_1 == GPIO_PIN_SET || feel_2 == GPIO_PIN_SET ||
                         feel_3 == GPIO_PIN_SET || feel_4 == GPIO_PIN_SET)
        {
            hitting = 1;
					test = 1;
        }
			}

				if(hitting == 1)
				{flag_out = 1;}
        if (flag_out == 1)
        {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
					vTaskDelay(1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
					vTaskDelay(1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
					vTaskDelay(800);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
					vTaskDelay(1);
										HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
					vTaskDelay(1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
					vTaskDelay(500);
					  flag_out = 0;
            hitting = 2;
					test = 0;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
    }
}

TaskHandle_t Remote_JY61_Handle;
   void Remote_JY61(void *pvParameters)
  {
      TickType_t last_wake_time = xTaskGetTickCount();

      static float gyro_z_filt   = 0.0f;
      static float slip_filt     = 0.0f;
      static float corr_filt     = 0.0f;

      vTaskDelay(pdMS_TO_TICKS(150));

      for(;;)
      {
          float gyro_z_raw = JY61.AngularVelocity.Z;
          gyro_z_filt = lowpass_filter(gyro_z_raw, &gyro_z_filt, GYRO_LPF_ALPHA);
          float gyro_z = gyro_z_filt;

          if (fabsf(gyro_z) < GYRO_DEADZONE) {
              gyro_z = 0.0f;
          }

          float expected_yaw = Remote_Control.Eomega * MAX_ROBOT_OMEGA * 180.0f / PI;

          float slip_raw = gyro_z - expected_yaw;
          slip_filt = lowpass_filter(slip_raw, &slip_filt, SLIP_LPF_ALPHA);
          float slip = slip_filt;

          if (fabsf(slip) < SLIP_DEADZONE) {
              slip = 0.0f;
          }

          gyro_slip_val = slip;
          slip_flag = (fabsf(slip) > SLIP_THRESHOLD) ? 1 : 0;

          if (slip_flag) {
              JY61_adjust.Kp = 1.3f;
              JY61_adjust.Kd = 0.5f;
              JY61_adjust.Ki = 0.0003f;
          } else {
              JY61_adjust.Kp = 0.8f;
              JY61_adjust.Kd = 0.35f;
              JY61_adjust.Ki = 0.0008f;
          }

          PID_Control2(slip, 0.0f, &JY61_adjust);
          float out = JY61_adjust.pid_out;

          if (out >  CORR_OUT_MAX) { out =  CORR_OUT_MAX; }
          if (out < -CORR_OUT_MAX) { out = -CORR_OUT_MAX; }

          if (fabsf(out) < CORR_OUT_DEADZONE) {
              out = 0.0f;
              JY61_adjust.error_inter = 0.0f;
          }

          corr_filt = lowpass_filter(out, &corr_filt, CORR_LPF_ALPHA);
          Wz_correction = corr_filt;

          vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
      }
  }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
if(ID == 0x201)
{
	int c =		Motor3508Recv(&steer_1,hcan, ID, Recv);
}
	if(ID == 0x202)
{
	int c =		Motor3508Recv(&steer_2,hcan, ID, Recv);

}if(ID == 0x203)
{
	int c =		Motor3508Recv(&steer_3,hcan, ID, Recv);
}
    }


