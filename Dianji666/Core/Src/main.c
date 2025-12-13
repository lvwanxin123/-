/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Speed Feedback PID
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
    float Kp;
    float Ki;
    float Kd;
    float error;
    float last_error;
    float integral;
    float output;
    float max_output;
    float max_integral;
} PID_TypeDef;

typedef struct 
{
    int16_t speed_rpm;
    int16_t real_current;
    int16_t angle;      
    uint8_t temp;
    
    int16_t last_angle;  
    int32_t total_angle; 
    int32_t loop_count;  
} Motor_TypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MOTOR_ID            0x200 // 发送ID
#define MOTOR_FEEDBACK_ID   0x203 // 反馈ID (对应 ID 3)
#define MAX_CURRENT         16384 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

Motor_TypeDef motor_info;
PID_TypeDef pos_pid; 

int32_t target_position = 0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Set_Motor_Current(int16_t current);
void CAN_Filter_Config(void);  
void PID_Init(PID_TypeDef *pid);

float PID_Calc(PID_TypeDef *pid, float target, float feedback, float current_speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  
  /* USER CODE BEGIN 2 */

  // 1. 初始化 PID
  PID_Init(&pos_pid);
  
  // 2. 初始化电机状态
  motor_info.loop_count = 0;
  motor_info.total_angle = 0;
  motor_info.last_angle = -1; 
  
  // 3. 启动 CAN
  CAN_Filter_Config();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


  while (motor_info.last_angle == -1) 
  {
      HAL_Delay(1); 
  }
  

  target_position = motor_info.total_angle;
  pos_pid.error = 0.0f;
  pos_pid.last_error = 0.0f;
  pos_pid.integral = 0.0f;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      float current_output = PID_Calc(&pos_pid, 
                                      (float)target_position, 
                                      (float)motor_info.total_angle,
                                      (float)motor_info.speed_rpm);
                                      
      Set_Motor_Current((int16_t)current_output); 
     
 
      HAL_Delay(1); 
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief 
 */
void Set_Motor_Current(int16_t current) {
    TxHeader.StdId = MOTOR_ID; 
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    if (current > MAX_CURRENT) current = MAX_CURRENT;
    if (current < -MAX_CURRENT) current = -MAX_CURRENT;

    TxData[0] = 0;
    TxData[1] = 0;
    TxData[2] = 0; 
    TxData[3] = 0;
    

    TxData[4] = (uint8_t)(current >> 8);
    TxData[5] = (uint8_t)(current);
    
    TxData[6] = 0; 
    TxData[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

/**
 * @brief CAN 过滤器配置
 */
void CAN_Filter_Config(void) 
{
    CAN_FilterTypeDef can_filter;
    
    can_filter.FilterActivation = ENABLE;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;  
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterBank = 0;
    
    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
}

/**
 * @brief 初始化 PID 参数 (针对速度反馈方案的参数)
 */
void PID_Init(PID_TypeDef *pid) {
    
 
    pid->Kp = 3.5f;     
    
    pid->Ki = 0.0f;     

    pid->Kd = 4.0f;     
    
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    

    pid->max_output = 16384.0f;   
    pid->max_integral = 2000.0f;  
}

/**
 * @brief PID 计算函数 (速度反馈版)
 */
float PID_Calc(PID_TypeDef *pid, float target, float feedback, float current_speed) {
    pid->error = target - feedback;
	
	if (fabs(pid->error) < 20.0f) {
        pid->error = 0.0f;
   
    }
  
    pid->integral += pid->error;

    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;


    pid->output = (pid->Kp * pid->error) + 
                  (pid->Ki * pid->integral) - 
                  (pid->Kd * current_speed);
    


    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;
    
    return pid->output;
}

/**
 * @brief CAN 接收中断回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

        if (RxHeader.StdId == MOTOR_FEEDBACK_ID) { 
            
            int16_t now_angle = (RxData[0] << 8) | RxData[1];
            

            motor_info.speed_rpm = (int16_t)((RxData[2] << 8) | RxData[3]);
            
            motor_info.real_current = (RxData[4] << 8) | RxData[5];
            motor_info.temp = RxData[6];

            if (motor_info.last_angle != -1) {
                int diff = now_angle - motor_info.last_angle;
                
                if (diff < -4096) {
                    motor_info.loop_count++;
                } 
                else if (diff > 4096) {
                    motor_info.loop_count--;
                }
            }
            
            motor_info.last_angle = now_angle;
            motor_info.total_angle = (motor_info.loop_count * 8192) + now_angle;
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
