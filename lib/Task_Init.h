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
#include "cylinder.h"
#define Remote_BT_0_WIFI_1 1

#if !Remote_BT_0_WIFI_1
#pragma pack(1)
typedef struct
{
  uint16_t Left_Key_Up : 1;
  uint16_t Left_Key_Down : 1;
  uint16_t Left_Key_Left : 1;
  uint16_t Left_Key_Right : 1;
  uint16_t Left_Rocker : 1;
  uint16_t Left_Encoder : 1;
  uint16_t Left_Switch_Up : 1;
  uint16_t Left_Switch_Down : 1;  
  uint16_t Right_Key_Up : 1;
  uint16_t Right_Key_Down : 1;
  uint16_t Right_Key_Left : 1;
  uint16_t Right_Key_Right : 1;
  uint16_t Right_Rocker : 1;
  uint16_t Right_Encoder : 1;
  uint16_t Right_Switch_Up : 1;
  uint16_t Right_Switch_Down : 1;  
  } hw_key_t;

typedef struct {
	uint8_t head;
	int16_t rocker[4];
	hw_key_t Key;
	uint32_t Left_Encoder;
	uint32_t Right_Encoder;
  uint16_t crc;
} UART_DataPack;
#pragma pack()

#else

#pragma pack(1)
typedef struct{
   uint8_t Left_Key_Up : 1;         
   uint8_t Left_Key_Down : 1;       
   uint8_t Left_Key_Left : 1;       
   uint8_t Left_Key_Right : 1;       
   uint8_t Left_Switch_Up_or_Left : 1;       
   uint8_t Left_Switch_Down_or_Right: 1;       
   uint8_t UNUSED1 : 1;
   uint8_t UNUSED2 : 1;

   uint8_t Right_Key_Up : 1;        
   uint8_t Right_Key_Down : 1;      
   uint8_t Right_Key_Left : 1;      
   uint8_t Right_Key_Right : 1;     
   uint8_t Right_Switch_Up_or_Right : 1;      
   uint8_t Right_Switch_Down_or_Left : 1;      
   uint8_t UNUSED3 : 1; 
   uint8_t UNUSED4 : 1;
} hw_key_t;
  
//遥控模式（蓝牙）
typedef struct {
	uint8_t head;
	int16_t rocker[4];
	hw_key_t Key;
	uint8_t end;
} UART_DataPack;

#pragma pack()
#endif

typedef struct {
    int16_t Ex;
    int16_t Ey;
    int16_t Eomega;
	int16_t mode;
    hw_key_t *Key_Control;
    hw_key_t First,Second;
} Remote_Handle_t;

typedef enum{
    STP,
    STOP,
    REMOTE,
    AUTO,
}ChassisMode;

extern uint8_t usart4_dma_buff[30]; //串口接收数据

extern UART_DataPack RemoteData;  //将串口接收的数据存到这里
extern Remote_Handle_t Remote_Control; //取出遥控器数据
extern ChassisMode chassis_mode;

void Updatakey(Remote_Handle_t * xx);
void Move_Task(void *pvParameters);
void Task_Init(void);
extern TaskHandle_t Hit_Task_Handle;
extern void Hit_Task(void *pvParameters);

#endif
