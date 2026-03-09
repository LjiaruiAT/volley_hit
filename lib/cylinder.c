#include "cylinder.h"

void cylinder_up(void)
{	
GPIO_PinState GPIOA5_State = GPIO_PIN_RESET;
GPIO_PinState GPIOA7_State = GPIO_PIN_RESET;
GPIO_PinState GPIOE9_State = GPIO_PIN_RESET;
}
void cylinder_down(void)
{
GPIO_PinState GPIOA5_State = GPIO_PIN_SET;
GPIO_PinState GPIOA7_State = GPIO_PIN_SET;
GPIO_PinState GPIOE9_State = GPIO_PIN_SET;
}
void cylinder_Init(void)
{
GPIO_PinState GPIOA5_State = GPIO_PIN_SET;
GPIO_PinState GPIOA7_State = GPIO_PIN_SET;
GPIO_PinState GPIOE9_State = GPIO_PIN_SET;
}








