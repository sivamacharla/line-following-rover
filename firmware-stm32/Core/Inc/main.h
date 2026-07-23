/* USER CODE BEGIN Header */
/**
  * Line-following rover firmware — STM32 Nucleo-F401RE port.
  * See firmware-stm32/README.md for CubeMX pin/peripheral configuration.
  * This file assumes a CubeIDE-generated project skeleton; only the
  * application-relevant declarations are shown here.
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* USER CODE BEGIN Private defines */

// GPIO pin defines (match the CubeMX pin table in README.md)
#define TRIG_GPIO_Port   GPIOA
#define TRIG_Pin         GPIO_PIN_9

#define ECHO_GPIO_Port   GPIOC
#define ECHO_Pin         GPIO_PIN_7

#define L_IN1_GPIO_Port  GPIOB
#define L_IN1_Pin        GPIO_PIN_6
#define L_IN2_GPIO_Port  GPIOB
#define L_IN2_Pin        GPIO_PIN_4

#define R_IN3_GPIO_Port  GPIOB
#define R_IN3_Pin        GPIO_PIN_10
#define R_IN4_GPIO_Port  GPIOB
#define R_IN4_Pin        GPIO_PIN_5

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
