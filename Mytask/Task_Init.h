#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_
#include "RMLibHead.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "CANDrive.h"
#include "RobStride2.h"
#include "usart.h"
#include "bsp_dwt.h"
#include "Chassis.h"
#include "math.h"
#include "PID_old.h"
#include "math.h"
#include "motor.h"
#include "motorEx.h"
#include "comm_stm32_hal_middle.h"
#include "JY61.h"
#include "VESC.h"
#include "pid.h"
#include "AutoPilot.h"
#define PI 3.14159265359f
#define MAX_VELOCITY 10.0f	  // 底盘最大速度
#define MAX_OMEGA PI*10	 	 //最大角速度
#define LENGTH 0.457f	 	//底盘中心到轮子的距离
#define WHEEL_RADIUS 0.075f  //轮的半径
#define MODE_t  1		  //等于0为漫反射开关模式，1为摄像头模式
#define ANGLE2RAD(x) (x) * PI / 180.0f
#define MAX_ROBOT_VEL 5.0f // m/s
#define MAX_ROBOT_OMEGA ANGLE2RAD(30.0f)

typedef struct{
	uint8_t Left_Key_Up;         
	uint8_t Left_Key_Down;       
	uint8_t Left_Key_Left;       
	uint8_t Left_Key_Right;       
	uint8_t Left_Switch_Up;       
	uint8_t Left_Switch_Down;
	uint8_t Left_Broadside_Key;

	uint8_t Right_Key_Up;        
	uint8_t Right_Key_Down;      
	uint8_t Right_Key_Left;      
	uint8_t Right_Key_Right;     
	uint8_t Right_Switch_Up;      
	uint8_t Right_Switch_Down;      
	uint8_t Right_Broadside_Key;
} hw_key_t;
  


typedef struct {
    float Ex;
    float Ey;
    float Eomega;
    hw_key_t First,Second;
} Remote_Handle_t;

typedef enum{
    STP,
    STOP,
    REMOTE,
    AUTO,
}ChassisMode;;

typedef struct
{
	PID2 PID;
	int dead_area;
	PID_EREOR_TypeDef PID_ERROR;
	VESC_t steer;
}VESC_INIT;
//extern uint8_t usart4_dma_buff[30]; //串口接收数据
//extern UART_DataPack RemoteData;  //将串口接收的数据存到这里
//extern Remote_Handle_t Remote_Control; //取出遥控器数据
extern ChassisMode chassis_mode;

void Task_Init(void);
void Remote_Jy61(void *pvParameters);
void Remote_Analysis();
void Remote(void *pvParameters);

extern TaskHandle_t Hit_Task_Handle;
extern void Hit_Task(void *pvParameters);
#define KEY_RISING_EDGE(cur, last, field)  ((cur.field == 1) && (last.field == 0))
#endif
