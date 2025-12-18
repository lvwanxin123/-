/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 基于位置 + 速度双环的全向轮底盘示例
  ******************************************************************************
  * @attention
  *
  * 该示例展示了 4 个 M3508 电机在 CAN 总线上运行的“双环 PID（位置外环 + 速度内环）”
  * 控制方式，并给出了麦克纳姆/全向轮底盘的运动学求解。通过充分的注释，便于新手
  * 理解并根据需求快速修改（例如更换几何参数、调整 PID、替换输入速度指令等）。
  *
  * 主要流程：
  * 1. 接收底盘期望速度 (vx, vy, wz) -> 运动学求解出 4 轮目标转速 (rpm)
  * 2. 将目标转速积分为“目标位置”脉冲数 -> 位置 PID -> 给出目标转速微调量
  * 3. 速度 PID 根据目标/反馈转速计算驱动电流 -> 通过 CAN 发送到电机
  *
  * 提示：若您使用电调内置的电流环，可以将速度 PID 的输出直接当作“目标转矩/电流”
  * 发送；若使用 FOC/驱动库，也可在 Set_Motor_Current 中替换为对应接口。
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
#ifndef M_PI
#define M_PI 3.1415926f
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    // --- PID 参数（按需修改） ---
    float Kp;            // 比例系数：误差越大输出越强
    float Ki;            // 积分系数：消除稳态误差，过大易抖动
    float Kd;            // 微分系数：抑制快速变化，提升响应稳定性

    // --- 运行时变量（无需手动改动） ---
    float error;         // 当前误差 e(k)
    float last_error;    // 上一次误差 e(k-1)
    float integral;      // 积分累加项
    float output;        // PID 输出

    // --- 保护限幅 ---
    float max_output;    // 输出限幅（防止超过电流/速度能力）
    float max_integral;  // 积分限幅（防止积分饱和）
} PID_TypeDef;

typedef struct
{
    int16_t speed_rpm;      // 实际转速 (rpm)
    int16_t real_current;   // 实际电流反馈（如驱动支持）
    int16_t angle;          // 编码器当前角度 [0,8191]
    uint8_t temp;           // 温度（若驱动反馈）

    int16_t last_angle;     // 上一次角度，用于圈计数（-1 表示尚未收到数据）
    int32_t total_angle;    // 叠加圈数后的绝对角度（单位：编码器脉冲）
    int32_t loop_count;     // 圈数计数（正负代表转向）
} Motor_TypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_TX_ID         0x200    // 发送到电机的标准 ID（0x200 下发 4 个电机电流）
#define MOTOR_ID_BASE       0x201    // 电机反馈 ID 起始值 (0x201~0x204)
#define MOTOR_COUNT         4        // 全向轮数量
#define MAX_CURRENT         16384    // M3508 电流指令最大值（16bit 有符号）
#define ENCODER_RESOLUTION  8192.0f  // M3508 编码器每圈脉冲数
#define CONTROL_PERIOD_S    0.001f   // 控制周期：1ms（示例用 HAL_Delay(1) 实现，实机建议定时器/RTOS 定时）
#define WHEEL_RADIUS_M      0.076f   // 轮半径（米）-> TODO: 根据实际轮子测量填写
#define LX_M                0.15f    // 车体几何：中心到轮的 x 方向半距 -> TODO: 用尺测量
#define LY_M                0.15f    // 车体几何：中心到轮的 y 方向半距 -> TODO: 用尺测量
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

Motor_TypeDef motors[MOTOR_COUNT];      // 4 个电机的反馈信息
PID_TypeDef   pos_pid[MOTOR_COUNT];     // 位置环 PID（外环）
PID_TypeDef   speed_pid[MOTOR_COUNT];   // 速度环 PID（内环）

float target_position[MOTOR_COUNT];     // 目标位置（编码器脉冲）
float target_speed[MOTOR_COUNT];        // 目标转速（rpm）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Set_Motor_Current(int16_t i1, int16_t i2, int16_t i3, int16_t i4);
void CAN_Filter_Config(void);
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_limit, float integral_limit);
float PID_Calc(PID_TypeDef *pid, float target, float feedback);
void Kinematics_Inverse(float vx, float vy, float wz, float wheel_rpm[4]);
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

  // 1. 初始化 4 个 PID（位置环与速度环参数可按实际调试修改）
  //    建议：先单独调试速度环，再引入位置环微调；输出/积分限幅避免抖动与饱和
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
      PID_Init(&pos_pid[i],   1.0f, 0.0f, 0.0f, 3000.0f, 5000.0f); // 位置环：输出目标速度 (rpm)
      PID_Init(&speed_pid[i], 5.0f, 0.1f, 0.0f, MAX_CURRENT, 3000.0f); // 速度环：输出目标电流
  }

  // 2. 初始化电机状态变量（圈数、角度、转速、电流等）
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
      motors[i].loop_count  = 0;
      motors[i].total_angle = 0;
      motors[i].last_angle  = -1; // -1 表示尚未收到过反馈
      motors[i].speed_rpm   = 0;
      motors[i].real_current= 0;
      motors[i].angle       = 0;
      motors[i].temp        = 0;
  }

  // 3. 启动 CAN（打开滤波与中断）：
  //    - Filter 全开：接受所有 ID，便于调试；正式使用可只放行 0x201~0x204
  //    - 开启 FIFO0 接收中断，在回调中更新编码器/转速/电流反馈
  CAN_Filter_Config();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // 4. 等待首次电机反馈，以获取初始角度
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
      while (motors[i].last_angle == -1)
      {
          HAL_Delay(1);
      }
      target_position[i] = (float)motors[i].total_angle;
      target_speed[i]    = 0.0f;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // ========== 1. 根据底盘目标速度 (m/s, rad/s) 计算每个轮子的期望转速 ==========
      // TODO: 将 vx_cmd、vy_cmd、wz_cmd 替换为遥控/上位机/路径规划结果
      //      前进/后退 -> vx，平移 -> vy，原地旋转 -> wz（麦克纳姆全向移动）
      float vx_cmd = 0.3f;   // x 方向线速度 (m/s)：前进为正
      float vy_cmd = 0.0f;   // y 方向线速度 (m/s)：左移为正
      float wz_cmd = 0.0f;   // 角速度 (rad/s)：逆时针为正

      float wheel_rpm[MOTOR_COUNT] = {0};
      Kinematics_Inverse(vx_cmd, vy_cmd, wz_cmd, wheel_rpm);

      // ========== 2. 将目标转速积分为“目标位置”，做位置外环 ==========
      // 积分思路：rpm -> rps -> 每周期脉冲增量，累加得到目标总角度（脉冲）
      // 位置环作用：抑制累计误差，防止长时间运行后偏移（适合低速/匀速场景）
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
          float rps          = wheel_rpm[i] / 60.0f;                           // 转/秒
          float pulse_inc    = rps * ENCODER_RESOLUTION * CONTROL_PERIOD_S;    // 本周期应增加的脉冲
          target_position[i] += pulse_inc;                                     // 累加目标位置

          // 位置环输出一个“目标速度补偿量”，用于逼近目标位置
          float pos_out      = PID_Calc(&pos_pid[i], target_position[i], (float)motors[i].total_angle);
          target_speed[i]    = wheel_rpm[i] + pos_out; // 期望速度 = 运动学速度 + 位置校正
      }

      // ========== 3. 速度内环：根据速度误差计算目标电流 ==========
      // 速度环输出可直接作为：
      //   a) FOC 库的电流/转矩命令，或
      //   b) M3508 电调允许的电流指令（本示例发送到 0x200）
      int16_t curr_cmd[MOTOR_COUNT];
      for (int i = 0; i < MOTOR_COUNT; i++)
      {
          float current_output = PID_Calc(&speed_pid[i], target_speed[i], (float)motors[i].speed_rpm);
          curr_cmd[i] = (int16_t)current_output;
      }

      // ========== 4. 将 4 路电流通过 CAN 下发到电机 ==========
      Set_Motor_Current(curr_cmd[0], curr_cmd[1], curr_cmd[2], curr_cmd[3]);

      // 控制循环节拍：示例用 HAL_Delay(1) -> 1ms；
      // 若使用硬件定时器或 RTOS，请把核心控制逻辑放入对应回调/任务，并保持周期与 CONTROL_PERIOD_S 一致
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
 * @brief 发送 4 个电机的电流指令（适配 DJI/M3508 协议：StdId=0x200，8 字节）
 * @param i1~i4 四个电机的目标电流（有符号 16 位）
 * @note  数据格式（大端）：
 *        Byte0-1: 电机1, Byte2-3: 电机2, Byte4-5: 电机3, Byte6-7: 电机4
 *        例：i1 高 8 位放入 Byte0，低 8 位放入 Byte1
 * @tip   若只驱动 2 个电机，可将剩余填 0；若使用其他协议，请修改 StdId/DLC/字节顺序
 */
void Set_Motor_Current(int16_t i1, int16_t i2, int16_t i3, int16_t i4) {
    TxHeader.StdId = MOTOR_TX_ID;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    // 输出限幅，防止超过驱动允许值，保护电机/驱动
    int16_t currents[4] = {i1, i2, i3, i4};
    for (int i = 0; i < 4; i++)
    {
        if (currents[i] >  MAX_CURRENT) currents[i] =  MAX_CURRENT;
        if (currents[i] < -MAX_CURRENT) currents[i] = -MAX_CURRENT;
    }

    TxData[0] = (uint8_t)(currents[0] >> 8);
    TxData[1] = (uint8_t)(currents[0]);
    TxData[2] = (uint8_t)(currents[1] >> 8);
    TxData[3] = (uint8_t)(currents[1]);
    TxData[4] = (uint8_t)(currents[2] >> 8);
    TxData[5] = (uint8_t)(currents[2]);
    TxData[6] = (uint8_t)(currents[3] >> 8);
    TxData[7] = (uint8_t)(currents[3]);

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

/**
 * @brief CAN 过滤器配置
 * @note  此处使用“全开掩码”，方便调试；正式项目可按需限定 StdId 范围
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
 * @brief 初始化 PID 参数
 * @param pid            PID 结构体指针
 * @param kp/ki/kd       PID 系数
 * @param out_limit      输出限幅（电流/速度）
 * @param integral_limit 积分限幅
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_limit, float integral_limit) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->max_output = out_limit;
    pid->max_integral = integral_limit;
}

/**
 * @brief PID 计算函数
 * @param target   目标值
 * @param feedback 反馈值
 * @return         PID 输出
 * @note 位置环/速度环通用，若需防抖可在调用前做死区处理
 */
float PID_Calc(PID_TypeDef *pid, float target, float feedback) {
    // 误差计算
    pid->error = target - feedback;

    // 积分环节（带防饱和）
    pid->integral += pid->error;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    // 微分项：用当前误差与上一时刻误差的差分近似（简单 D，若有噪声可做滤波）
    float derivative = pid->error - pid->last_error;
    pid->last_error = pid->error;

    pid->output = (pid->Kp * pid->error) +
                  (pid->Ki * pid->integral) +
                  (pid->Kd * derivative / CONTROL_PERIOD_S); // 近似 d/dt

    // 输出限幅
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    return pid->output;
}

/**
 * @brief 全向/麦克纳姆轮运动学逆解：底盘速度 -> 轮速
 * @param vx      车体 x 方向线速度 (m/s)，前进为正
 * @param vy      车体 y 方向线速度 (m/s)，左移为正
 * @param wz      车体角速度 (rad/s)，逆时针为正
 * @param wheel_rpm[4] 输出的 4 轮转速 (rpm)，顺序 FL, FR, BL, BR
 * @note 公式来源：标准麦克纳姆几何模型，保持符号一致即可“全向”移动
 */
void Kinematics_Inverse(float vx, float vy, float wz, float wheel_rpm[4])
{
    // 线速度 -> 轮角速度 (rad/s)
    float w_fl = ( vx - vy - (LX_M + LY_M) * wz) / WHEEL_RADIUS_M; // Front-Left
    float w_fr = ( vx + vy + (LX_M + LY_M) * wz) / WHEEL_RADIUS_M; // Front-Right
    float w_bl = ( vx + vy - (LX_M + LY_M) * wz) / WHEEL_RADIUS_M; // Back-Left
    float w_br = ( vx - vy + (LX_M + LY_M) * wz) / WHEEL_RADIUS_M; // Back-Right

    const float radps_to_rpm = 60.0f / (2.0f * (float)M_PI);
    wheel_rpm[0] = w_fl * radps_to_rpm;
    wheel_rpm[1] = w_fr * radps_to_rpm;
    wheel_rpm[2] = w_bl * radps_to_rpm;
    wheel_rpm[3] = w_br * radps_to_rpm;
}

/**
 * @brief CAN 接收中断回调函数
 * @note 解析 DJI/M3508 反馈帧：StdId=0x201~0x204，数据格式
 *       Byte0-1: 编码器角度，Byte2-3: 转速 rpm，Byte4-5: 实际电流，Byte6: 温度
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

        // 反馈 ID 范围：0x201~0x204，对应 1~4 号电机
        if (RxHeader.StdId >= MOTOR_ID_BASE && RxHeader.StdId < MOTOR_ID_BASE + MOTOR_COUNT) {
            uint8_t idx = (uint8_t)(RxHeader.StdId - MOTOR_ID_BASE); // 0~3 对应 FL/FR/BL/BR

            int16_t now_angle = (int16_t)((RxData[0] << 8) | RxData[1]);

            motors[idx].speed_rpm    = (int16_t)((RxData[2] << 8) | RxData[3]);
            motors[idx].real_current = (int16_t)((RxData[4] << 8) | RxData[5]);
            motors[idx].temp         = RxData[6];

            // 处理编码器绕圈：若角度跳变超过半圈，则认为过零点
            if (motors[idx].last_angle != -1) {
                int diff = now_angle - motors[idx].last_angle;
                if (diff < -4096) {
                    motors[idx].loop_count++;
                }
                else if (diff > 4096) {
                    motors[idx].loop_count--;
                }
            }

            motors[idx].last_angle  = now_angle;
            motors[idx].total_angle = (motors[idx].loop_count * 8192) + now_angle;
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
