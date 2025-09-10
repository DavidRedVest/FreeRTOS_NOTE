/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdio.h>

void MX_FREERTOS_Init(void); 
void StartDefaultTask(void *argument);

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128,
  .priority = (osPriority_t) osPriorityNormal,
};

void MX_FREERTOS_Init(void) {
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	if (defaultTaskHandle == NULL) {
		printf("osThreadNew error!\r\n");
	}
}


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
//    osDelay(500);
//	  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
	  //HAL_Delay(500);
	  
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	  //osDelay(500);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	  //osDelay(500);
	  HAL_Delay(500);
  }
  /* USER CODE END StartDefaultTask */
}


