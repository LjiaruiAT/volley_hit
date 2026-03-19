#include "Chassis.h"
#include "VESC.h"
#include "PID_old.h"
#include "PID.h"
PID_EREOR_TypeDef PID_v1;
PID_EREOR_TypeDef PID_v2;
PID_EREOR_TypeDef PID_v3;
 //电机驱动
VESC_t steering1={
	.motor_id=0x01,
	.hcan = &hcan2,

  
};
VESC_t steering2={ 
	.motor_id=0x02,
	.hcan = &hcan2,

};
VESC_t steering3={
	.motor_id=0x03,
	.hcan = &hcan2,
         
};
int dead_v1 = 5;
int dead_v2 = 0;
int dead_v3 = 0;

PID2 vesc1;
PID2 vesc2;
PID2 vesc3;
uint8_t flag = 0;
float Vx =0;   //前后移动
float Vy =0;   //左右移动
float Wz =0;   //顺逆自转
//该变量的值可能会被程序外的因素（如硬件、其他线程）修改
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

	vesc1.Kp =7.0f;
	vesc1.Ki = 0.0f;
	vesc1.Kd = 40.0f;
	vesc1.limit = 10000.0f;
	vesc1.output_limit = 40.0f;
	vesc2.Kp =7.0f;
	vesc2.Ki = 0.0f;
	vesc2.Kd = 40.0f;
	vesc2.limit = 10000.0f;
	vesc2.output_limit = 40.0f;
	vesc3.Kp =7.0f;
	vesc3.Ki = 0.0f;
	vesc3.Kd = 40.0f;
	vesc3.limit = 100000.0f;
	vesc3.output_limit = 40.0f;
    PID_EREOR_Init(&PID_v1,vesc1.Kp,vesc1.Ki,vesc1.Kd,vesc1.output_limit,vesc1.limit,dead_v1,0.3f);
   	PID_EREOR_Init(&PID_v2,vesc2.Kp,vesc2.Ki,vesc2.Kd,vesc2.output_limit,vesc2.limit,dead_v2,0.3f);
    PID_EREOR_Init(&PID_v3,vesc3.Kp,vesc3.Ki,vesc3.Kd,vesc3.output_limit,vesc3.limit,dead_v3,0.3f);

	for(;;)
	{
//		Vx = - remote_to_velocity(&recv_pack.Ey);
//		Vy = - remote_to_velocity(&recv_pack.Ex);
//        Wz =   remote_to_Omega(&recv_pack.Eomega);

		v1 = -Vx*0.5f+ Vy*(sqrt (3.0f)/2.0)+LENGTH * Wz;
		v2 = -Vx*0.5f- Vy*(sqrt (3.0f)/2.0)+LENGTH * Wz;
        v3 =  Vx +LENGTH * Wz;

		wheel_one=  -(int16_t)((v1 / (2.0f * PI * WHEEL_RADIUS)) );
		wheel_two=  (int16_t)((v2 / (2.0f * PI * WHEEL_RADIUS)) );
		wheel_three=-(int16_t)((v3 /(2.0f * PI * WHEEL_RADIUS)) );
			
//		PID_Control2((float)(steering1.epm / 7.0f/(3.4f)), (wheel_one   ), &vesc1);
//		PID_Control2((float)(steering2.epm / 7.0f/(3.4f)), (wheel_two   ), &vesc2);
//		PID_Control2((float)(steering3.epm / 7.0f/(3.4f)), (wheel_three ), &vesc3);
        PID_EREOR_Calculate(&PID_v1,   wheel_one - (float)(steering1.epm / 7.0f/(3.4f)));
//        PID_EREOR_Calculate(&PID_v2,   wheel_two - (float)(steering2.epm / 7.0f/(3.4f)));
//        PID_EREOR_Calculate(&PID_v3,   wheel_three - (float)(steering3.epm / 7.0f/(3.4f)));
//		
		VESC_SetCurrent(&steering1, PID_v1.output);

//VESC_SetCurrent(&steering2, PID_v2.output);
//		VESC_SetCurrent(&steering3, PID_v3.output);

		vTaskDelayUntil(&xLastWakeTime,2);
	}
}

bool is_remote_active(void)
{
    return abs(recv_pack.mode) > 10;
}

float remote_to_velocity(int16_t *remote_value) {
    if (*remote_value == 0) 
	{
        return 0.0f;
    }
	int16_t rv=((float)*remote_value / 2047.0f) * MAX_VELOCITY;
    return rv;
}

float remote_to_Omega(int16_t *remote_value) {
    if (*remote_value == 0)
	{
        return 0.0f;
    }
	int16_t ro=((float)*remote_value / 2047.0f) * MAX_OMEGA;
    return ro;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	VESC_ReceiveHandler(&steering1, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&steering2, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&steering3, &hcan2, ID,Recv);
    }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
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
      Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff, 0);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
      __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}
