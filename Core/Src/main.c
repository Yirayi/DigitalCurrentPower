/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "KeyBoard.h"
#include "INA226.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//---------------------------------------------OLED-----------------------------------------------------
//typedef enum {
//	noneSelected = 0,
//    beforeDot1 = 1,
//    behindDot1 = 2,
//    behindDot2 = 3
//} IsetSelectedFlags;
//IsetSelectedFlags IFlags = noneSelected;

typedef enum { MODE_CC = 0, MODE_CV } WorkMode;
static WorkMode work_mode = MODE_CC;
float Uin = 0;
char msg1[20] = "";
char msg2[30] = "";
char msg3[20] = "";
char msg4[20] = "";
//----------------------------ADC----------------------------
uint16_t Vinadc = 0;
//INA226电压修正
#define INA226_VOLTAGE_SCALE  1.006f
//---------------------------------keyboard--------------------------------------------------------------
char Isetbuffer[4] = {0,0,0,0};
short Ibuffercounter = 0;
char key = 0;
//--------------------------------------控制--------------------------------------------------------------
#define PID_KP          50.0f
#define PID_KI          10.0f
#define PID_KD           5.0f
#define PID_TS           0.133f
#define PID_DUTY_MIN    0UL
#define PID_DUTY_MAX    1439UL
#define PID_INT_LIMIT   (PID_DUTY_MAX / PID_KI)
#define PID_D_ALPHA      0.3f
#define DEFAULT_DUTY    0.8

float error = 0;
static float pid_integral  = 0.0f;
static float prev_error    = 0.0f;
static float d_filtered    = 0.0f;

void PID_Reset(void)
{
    pid_integral = 0.0f;
    prev_error   = 0.0f;
    d_filtered   = 0.0f;
}


float Iset = 1.0;
static uint32_t pwm_duty = PID_DUTY_MAX;
//-------------------------------------控制函数-------------------------------------------------------------
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static void Set_PWM_Duty(uint32_t duty)
{
    if (duty > PID_DUTY_MAX) duty = PID_DUTY_MAX;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
    pwm_duty = duty;
}
void PID_Update(float current_A)
{
    /* ---------- 误差 ---------- */
    float error = Iset - current_A;

    /* ---------- 积分项：梯形/矩形积分 + 限幅防饱和 ---------- */
    pid_integral += error * PID_TS;
    pid_integral  = clampf(pid_integral, -PID_INT_LIMIT, PID_INT_LIMIT);

    /* ---------- 微分项：差分 + 一阶低通滤波 ----------
     *   d_filtered = α·d_raw + (1-α)·d_filtered_prev
     *   α=0.3：仅 30% 权重给新值，有效抑制高频噪声
     * ------------------------------------------------- */
    float d_raw  = (error - prev_error) / PID_TS;
    d_filtered   = PID_D_ALPHA * d_raw + (1.0f - PID_D_ALPHA) * d_filtered;
    prev_error   = error;                  /* 更新历史误差 */

    /* ---------- 输出合成 ---------- */
    float output = PID_KP * error
                 + PID_KI * pid_integral
                 + PID_KD * d_filtered;

    /*
     * output > 0 → 电流不足 → 减小 duty → Vout 升高 → 电流上升
     * output < 0 → 电流过大 → 增大 duty → Vout 降低 → 电流下降
     */
    float new_duty = (float)pwm_duty - output;
    Set_PWM_Duty((uint32_t)clampf(new_duty, PID_DUTY_MIN, PID_DUTY_MAX));
}
static void CV_Update(float voltage_V)
{
	static float CVintegral = 0;

    float cverror = 18.0f - voltage_V;
    CVintegral += cverror * 0.133f;
    CVintegral = clampf(CVintegral, -200.0f, 200.0f);
    float new_duty = (int32_t)pwm_duty - (float)(cverror * 5.0f) - CVintegral*1;
    Set_PWM_Duty((uint32_t)clampf((float)new_duty, 0.0f, 719.0f));
}
void OledRefreshData(){
//	sprintf(msg1,"%.2fA",Iset);
//	sprintf(msg2,"Io:%.2fA Uo:%.2fV",ina226_data.current_A,ina226_data.voltage_V);
//	sprintf(msg3,"Rload:%.2f Ohm",ina226_data.resistance_Ohm);
//	sprintf(msg4,"Ui:%.2fV Mode:%s",Uin,work_mode ==MODE_CC? "CC":"CV");
//	OLED_NewFrame();
//	OLED_PrintString(0, 0 , "Iset:", &font16x16, OLED_COLOR_NORMAL);
//	OLED_PrintASCIIChar(40, 0 , msg1[0], &afont16x8, IFlags == beforeDot1?OLED_COLOR_REVERSED:OLED_COLOR_NORMAL);
//	OLED_PrintASCIIChar(55, 0 , msg1[2], &afont16x8, IFlags == behindDot1?OLED_COLOR_REVERSED:OLED_COLOR_NORMAL);
//	OLED_PrintASCIIChar(65, 0 , msg1[3], &afont16x8, IFlags == behindDot2?OLED_COLOR_REVERSED:OLED_COLOR_NORMAL);
//	OLED_PrintASCIIChar(48, 0 , '.'    , &afont16x8, OLED_COLOR_NORMAL);
	sprintf(msg1,"Iset%.2fA",Iset);
	sprintf(msg2,"Io:%.3fA Uo:%.2fV",ina226_data.current_A,ina226_data.voltage_V*INA226_VOLTAGE_SCALE);
	sprintf(msg3,"Rload:%.2f Ohm",ina226_data.resistance_Ohm);
	sprintf(msg4,"Ui:%.2fV Mode:%s",Vinadc * 6 *3.3f / 4095.0f,work_mode ==MODE_CC? "CC":"CV");
	OLED_NewFrame();
	OLED_PrintString(0, 0 , msg1, &font16x16, OLED_COLOR_NORMAL);

	OLED_PrintASCIIChar(80, 0 , Isetbuffer[0]=='/0'?'0':Isetbuffer[0], &afont16x8,OLED_COLOR_NORMAL);
	OLED_PrintASCIIChar(95, 0 , Isetbuffer[1]=='/0'?'0':Isetbuffer[1], &afont16x8, OLED_COLOR_NORMAL);
	OLED_PrintASCIIChar(110, 0 , Isetbuffer[2]=='/0'?'0':Isetbuffer[2], &afont16x8, OLED_COLOR_NORMAL);
	OLED_PrintASCIIChar(88, 0 , '.'    , &afont16x8, OLED_COLOR_NORMAL);

	OLED_PrintString(0, 20, msg2, &mediumfont, OLED_COLOR_NORMAL);
	OLED_PrintString(0, 35, msg3, &mediumfont, OLED_COLOR_NORMAL);
	OLED_PrintString(0, 50, msg4, &mediumfont, OLED_COLOR_NORMAL);
}
void OledKeyBoardDebug(){
	sprintf(msg1,"Iset%.2fA",Iset);

	OLED_NewFrame();
	OLED_PrintString(0, 0 , msg1, &font16x16, OLED_COLOR_NORMAL);
	OLED_PrintASCIIChar(40, 40 , Isetbuffer[0], &afont16x8,OLED_COLOR_NORMAL);
	OLED_PrintASCIIChar(55, 40 , Isetbuffer[1], &afont16x8, OLED_COLOR_NORMAL);
	OLED_PrintASCIIChar(65, 40 , Isetbuffer[2], &afont16x8, OLED_COLOR_NORMAL);
	OLED_PrintASCIIChar(48, 40 , '.'    , &afont16x8, OLED_COLOR_NORMAL);

}
void OledDebugRefreshData(){
	static int running = 0;
	int compare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);

	sprintf(msg1,"Iset:%.2fA  %.2f",Iset,error);
	sprintf(msg2,"Io:%.3fA Uo:%.2fV",ina226_data.current_A,ina226_data.voltage_V*INA226_VOLTAGE_SCALE);
	sprintf(msg3,"compare:%d",compare);
	sprintf(msg4,"%.3f",pid_integral);
	OLED_NewFrame();
	OLED_PrintString(0, 0 , msg1, &mediumfont, OLED_COLOR_NORMAL);
	OLED_PrintString(0, 15 , msg2, &mediumfont, OLED_COLOR_NORMAL);
	OLED_PrintString(0, 30 , msg3, &mediumfont, OLED_COLOR_NORMAL);
	OLED_PrintString(0, 45 , HAL_GPIO_ReadPin(IOEN_GPIO_Port, IOEN_Pin)==GPIO_PIN_SET?"ON":"OF", &mediumfont, OLED_COLOR_NORMAL);
	OLED_PrintString(50, 45 , msg4, &mediumfont, OLED_COLOR_NORMAL);

	running++;
	if(running==10){
		running =0;
	}
}
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  	//PWM输出
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, DEFAULT_DUTY*PID_DUTY_MAX);
	//ADC
	HAL_ADCEx_Calibration_Start(&hadc1);
	//其余外设初始化
	HAL_Delay(50);

	INA226_Init(&hi2c1);

	OLED_Init();

	HAL_StatusTypeDef KBstatus;
	KBstatus = Keyboard_Init(&hi2c2);
	initIset(Isetbuffer,&Ibuffercounter);
	uint32_t last_poll_tick = 0;

	//---------------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if (ina226_state == INA226_STATE_IDLE){
		INA226_StartRead();
	}
	if (ina226_data.data_ready){
		ina226_data.data_ready = 0;

		float I = ina226_data.current_A;
		float V = ina226_data.voltage_V;

		if (HAL_GPIO_ReadPin(IOEN_GPIO_Port, IOEN_Pin)==GPIO_PIN_SET){
			/* 判断工作模式 */
			if (V >= 18.0f){
				work_mode = MODE_CV;
				/* CV模式：维持18V */
				CV_Update(V);
				/* CV模式下积分清零，防止切回CC时积分饱和 */
				pid_integral = 0.0f;
			}
			else
			{
				work_mode = MODE_CC;
				/* CC模式：PID调节电流至Iset */
				PID_Update(I);
			}
		}
	}

	if (HAL_GetTick() - last_poll_tick >= KEYBOARD_POLL_INTERVAL){
		last_poll_tick = HAL_GetTick();

		/* -------- 方式 A：获取单个按键（最常用） -------- */
		char keybuffer = Keyboard_GetKey();
		if(key != keybuffer){
			key=keybuffer;//一次按下只取一次值
			if (key != '\0'){
				storeIset(key,&Ibuffercounter,Isetbuffer);//将键盘输入存入buffer

			}
		}
	}
	getIset(Isetbuffer,&Iset,&Ibuffercounter);//在确认后，将buffer中缓存的数据转换为数值

	//OledKeyBoardDebug();
	OledRefreshData();
	//OledDebugRefreshData();
	OLED_ShowFrame();
	HAL_GPIO_TogglePin(BoardLED_GPIO_Port, BoardLED_Pin);
	HAL_Delay(50);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* 转发给INA226驱动（内部会判断句柄是否匹配） */
    INA226_MemRxCallback(hi2c);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == AlertIn_Pin)  /* PB0 = Alert引脚 */
    {
    	if (ina226_state == INA226_STATE_IDLE)
		{
			INA226_StartRead();
		}
    }
    else if (GPIO_Pin == KeyDown_Pin)
    {
    	pid_integral == 0;
		HAL_GPIO_TogglePin(IOEN_GPIO_Port, IOEN_Pin);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
