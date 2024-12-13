/*
 * led.h
 *
 *  Created on: Oct 12, 2024
 *      Author: cary
 */

#ifndef BSP_LED_LED_H_
#define BSP_LED_LED_H_

/* LED 端口定义 */
#define LED1(x)  do{ x ? \
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET) : \
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); \
}while(0)

#define LED2(x)  do{ x ? \
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET) : \
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); \
}while(0)

#define LED3(x)  do{ x ? \
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET) : \
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); \
}while(0)

/* LED 取反定义 */
#define LED1_TOGGLE() do{ HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); }while(0)
#define LED2_TOGGLE() do{ HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); }while(0)
#define LED3_TOGGLE() do{ HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); }while(0)

#endif /* BSP_LED_LED_H_ */
