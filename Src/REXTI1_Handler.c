#include "main.h"
#include "REXTI1_Handler.h"

void EXTI1_IRQHandler(void)
{
    HAL_GPIO_TogglePin(GPIOB,LD_R_Pin);
}