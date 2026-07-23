/* USER CODE BEGIN Header */
/**
  * Line-following rover firmware — STM32 Nucleo-F401RE port (HAL).
  *
  * Same control strategy as firmware/rover_firmware.ino: 5-sensor IR array
  * for line position, HC-SR04 ultrasonic for obstacle/terrain-edge sensing,
  * EMA-filtered sensor fusion, and a PID loop (with filtered derivative +
  * output clamp — see simulation/rover_sim.py for why) driving a
  * differential motor pair via TIM3 PWM.
  *
  * See firmware-stm32/README.md for CubeMX pin/peripheral configuration
  * this file assumes. NOT compiled/flashed in this repo (no ARM toolchain
  * available where it was written) — build in STM32CubeIDE and expect to
  * fix any peripheral-init mismatches against your generated project.
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;   // free-running 1MHz counter, used for ultrasonic pulse timing
TIM_HandleTypeDef htim3;   // PWM: CH1 = left motor, CH2 = right motor
UART_HandleTypeDef huart2; // telemetry, routed to ST-LINK VCP

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PV */

// ---------- Tuning constants (from simulation/rover_sim.py grid search) ----------
static const float KP = 34.0f;
static const float KI = 0.6f;
static const float KD = 9.5f;
static const float D_FILTER_ALPHA = 0.25f;   // low-pass on derivative term (anti "derivative kick")
static const float PID_OUTPUT_LIMIT = 120.0f;

static const int   BASE_SPEED   = 600;   // PWM duty, range 0-999 (TIM3 ARR = 999)
static const int   MAX_SPEED    = 999;
static const int   MIN_SPEED    = 0;

static const float IR_EMA_ALPHA = 0.35f;
static const float US_EMA_ALPHA = 0.25f;

static const float OBSTACLE_SLOW_CM = 25.0f;
static const float OBSTACLE_STOP_CM = 8.0f;

static const uint32_t LOOP_INTERVAL_MS = 20; // 50 Hz control loop
static const uint32_t ECHO_TIMEOUT_US  = 30000;

static const uint32_t IR_ADC_CHANNELS[5] = {
  ADC_CHANNEL_0,  // PA0
  ADC_CHANNEL_1,  // PA1
  ADC_CHANNEL_4,  // PA4
  ADC_CHANNEL_8,  // PB0
  ADC_CHANNEL_11, // PC1
};
static const float SENSOR_WEIGHTS[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// ---------- State ----------
static float irFiltered[5] = {0, 0, 0, 0, 0};
static float lineError = 0.0f;
static float lastError = 0.0f;
static float integral = 0.0f;
static float filteredDerivative = 0.0f;
static float ultrasonicFiltered = 100.0f;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
static float emaFilter(float previous, float sample, float alpha);
static float readIRSensor(uint32_t channel);
static void  readAndFilterIR(void);
static float computeLineError(void);
static float readUltrasonicCm(void);
static float computePID(float error, float dt);
static int   speedCapFromDistance(float distanceCm);
static void  setMotor(GPIO_TypeDef *in1Port, uint16_t in1Pin,
                       GPIO_TypeDef *in2Port, uint16_t in2Pin,
                       uint32_t timChannel, int speed);
static void  driveMotors(float correction, int speedCap);
static void  logTelemetry(uint32_t nowMs, float distanceCm, int speedCap, float correction);
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  uint32_t lastLoopTime = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    uint32_t now = HAL_GetTick();
    if (now - lastLoopTime < LOOP_INTERVAL_MS) {
      continue;
    }
    float dt = (now - lastLoopTime) / 1000.0f;
    lastLoopTime = now;

    readAndFilterIR();
    float distanceCm = readUltrasonicCm();
    ultrasonicFiltered = emaFilter(ultrasonicFiltered, distanceCm, US_EMA_ALPHA);

    lineError = computeLineError();
    float correction = computePID(lineError, dt);

    int speedCap = speedCapFromDistance(ultrasonicFiltered);
    driveMotors(correction, speedCap);

    logTelemetry(now, distanceCm, speedCap, correction);
    /* USER CODE END WHILE */
  }
}

/* USER CODE BEGIN 4 */

static float emaFilter(float previous, float sample, float alpha) {
  return alpha * sample + (1.0f - alpha) * previous;
}

// Blocking single-channel ADC read, normalized to 0.0-1.0 (12-bit resolution).
static float readIRSensor(uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  uint32_t raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return raw / 4095.0f;
}

static void readAndFilterIR(void) {
  for (int i = 0; i < 5; i++) {
    float raw = readIRSensor(IR_ADC_CHANNELS[i]);
    irFiltered[i] = emaFilter(irFiltered[i], raw, IR_EMA_ALPHA);
  }
}

// Weighted-average line position from filtered IR array, in [-2, 2].
// 0 = centered, negative = line left of center, positive = right.
static float computeLineError(void) {
  float weightedSum = 0.0f;
  float total = 0.0f;

  for (int i = 0; i < 5; i++) {
    weightedSum += irFiltered[i] * SENSOR_WEIGHTS[i];
    total += irFiltered[i];
  }

  if (total < 0.3f) {
    // No line detected: keep turning toward where it was last seen.
    return lastError >= 0 ? 2.0f : -2.0f;
  }

  return weightedSum / total;
}

static float readUltrasonicCm(void) {
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < 2) { }

  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < 10) { }
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET) {
    if (__HAL_TIM_GET_COUNTER(&htim2) > ECHO_TIMEOUT_US) {
      return 400.0f; // timeout -> treat as "no obstacle"
    }
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET) {
    if (__HAL_TIM_GET_COUNTER(&htim2) > ECHO_TIMEOUT_US) {
      return 400.0f;
    }
  }
  uint32_t durationUs = __HAL_TIM_GET_COUNTER(&htim2);

  return durationUs * 0.0343f / 2.0f;
}

static float computePID(float error, float dt) {
  integral += error * dt;
  if (integral > 50.0f) integral = 50.0f;
  if (integral < -50.0f) integral = -50.0f;

  float rawDerivative = (dt > 0) ? (error - lastError) / dt : 0.0f;
  filteredDerivative = emaFilter(filteredDerivative, rawDerivative, D_FILTER_ALPHA);
  lastError = error;

  float output = (KP * error) + (KI * integral) + (KD * filteredDerivative);
  if (output > PID_OUTPUT_LIMIT) output = PID_OUTPUT_LIMIT;
  if (output < -PID_OUTPUT_LIMIT) output = -PID_OUTPUT_LIMIT;
  return output;
}

// Sensor fusion: ultrasonic reading gates max speed independent of steering.
static int speedCapFromDistance(float distanceCm) {
  if (distanceCm <= OBSTACLE_STOP_CM) return 0;
  if (distanceCm >= OBSTACLE_SLOW_CM) return BASE_SPEED;

  float ratio = (distanceCm - OBSTACLE_STOP_CM) / (OBSTACLE_SLOW_CM - OBSTACLE_STOP_CM);
  return (int)(BASE_SPEED * ratio);
}

static void setMotor(GPIO_TypeDef *in1Port, uint16_t in1Pin,
                      GPIO_TypeDef *in2Port, uint16_t in2Pin,
                      uint32_t timChannel, int speed) {
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;

  HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, timChannel, speed);
}

static void driveMotors(float correction, int speedCap) {
  int leftSpeed  = speedCap - (int)correction;
  int rightSpeed = speedCap + (int)correction;

  setMotor(L_IN1_GPIO_Port, L_IN1_Pin, L_IN2_GPIO_Port, L_IN2_Pin, TIM_CHANNEL_1, leftSpeed);
  setMotor(R_IN3_GPIO_Port, R_IN3_Pin, R_IN4_GPIO_Port, R_IN4_Pin, TIM_CHANNEL_2, rightSpeed);
}

// CSV over UART: time,error,correction,distance,speedCap — matches the
// dashboard's expected serial format. Formats floats manually (2 decimal
// places) to avoid depending on newlib-nano's float printf support.
static void appendFixed(char *buf, int *pos, float value, int decimals) {
  int scale = 1;
  for (int i = 0; i < decimals; i++) scale *= 10;

  long scaled = (long)(value * scale + (value >= 0 ? 0.5f : -0.5f));
  long intPart = scaled / scale;
  long fracPart = scaled % scale;
  if (fracPart < 0) fracPart = -fracPart;

  *pos += sprintf(buf + *pos, "%ld.%0*ld", intPart, decimals, fracPart);
}

static void logTelemetry(uint32_t nowMs, float distanceCm, int speedCap, float correction) {
  char buf[96];
  int pos = 0;
  pos += sprintf(buf + pos, "%lu,", (unsigned long)nowMs);
  appendFixed(buf, &pos, lineError, 3);
  buf[pos++] = ',';
  appendFixed(buf, &pos, correction, 2);
  buf[pos++] = ',';
  appendFixed(buf, &pos, distanceCm, 1);
  buf[pos++] = ',';
  pos += sprintf(buf + pos, "%d\r\n", speedCap);

  HAL_UART_Transmit(&huart2, (uint8_t *)buf, pos, 100);
}

/* USER CODE END 4 */

/**
  * SystemClock_Config: HSI -> PLL -> 84MHz HCLK (matches README's clock config).
  * CubeMX will generate its own version of this from the Clock Configuration
  * tab — keep whichever one actually matches the values you set there.
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  // Default channel config; readIRSensor() reconfigures per-read.
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

// Free-running 1MHz counter for ultrasonic pulse timing (mirrors Arduino pulseIn).
// Assumes TIM2 clock = 84MHz (APB1 timer clock, 2x PCLK1 since APB1 prescaler != 1).
static void MX_TIM2_Init(void)
{
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83; // 84MHz / (83+1) = 1MHz -> 1 tick = 1us
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
}

// PWM for both motors: 1kHz, duty range 0-999. Same APB1-clock assumption as TIM2.
static void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L_IN1_GPIO_Port, L_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L_IN2_GPIO_Port, L_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(R_IN3_GPIO_Port, R_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(R_IN4_GPIO_Port, R_IN4_Pin, GPIO_PIN_RESET);

  // TRIG (output)
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  // ECHO (input)
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  // Motor direction pins (outputs)
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = L_IN1_Pin;
  HAL_GPIO_Init(L_IN1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = L_IN2_Pin;
  HAL_GPIO_Init(L_IN2_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = R_IN3_Pin;
  HAL_GPIO_Init(R_IN3_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = R_IN4_Pin;
  HAL_GPIO_Init(R_IN4_GPIO_Port, &GPIO_InitStruct);

  // IR sensor ADC pins (analog)
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
