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
uint16_t Vinadc[10] = {0};

//INA226电压修正
#define INA226_VOLTAGE_SCALE  (0.9977f)
//---------------------------------keyboard--------------------------------------------------------------
char Isetbuffer[4] = {0,0,0,0};
short Ibuffercounter = 0;
char key = 0;
//--------------------------------------控制--------------------------------------------------------------
#define PID_KP_BASE      50.0f
#define PID_KI_BASE      20.0f
#define PID_KD_BASE       5.0f
#define PID_TS            0.133f
#define PID_DUTY_MIN      0UL
#define PID_DUTY_MAX      1439UL
#define PID_INT_LIMIT    (PID_DUTY_MAX / PID_KI_BASE)
#define PID_D_ALPHA       0.3f
#define DEFAULT_DUTY      0.8f

/* 增益调度参数 */
#define R_REF             5.0f    /* 参考电阻（量程中点），增益基准点 */
#define R_CLAMP_LOW       2.0f    /* 电阻下限：低于此值按0.8Ω计算，防止增益过小 */
#define R_CLAMP_HIGH     10.0f    /* 电阻上限：高于此值按10Ω计算，防止增益过大 */

/* 误差非线性增益参数 */
#define E_BOOST_HALF      0.15f   /* 误差达到此值时Kp增益为基准的1.5倍（半增益点）*/
#define E_BOOST_MAX       1.8f    /* Kp误差增益最大倍数，不超过1.8以防振荡 */

float error = 0;
static float pid_integral  = 0.0f;
static float prev_error    = 0.0f;
static float d_filtered    = 0.0f;

float CVintegral = 0;
float cverror;

void CVPID_Reset(){
	CVintegral = 0;
}
void PID_Reset(void)
{
    pid_integral = 0.0f;
    prev_error   = 0.0f;
    d_filtered   = 0.0f;
}


float Iset = 1.0;
static float pwm_duty = PID_DUTY_MAX;
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
/**
 * @brief  获取当前负载电阻估计值（带保护）
 * @return 钳位后的电阻值（Ω），用于增益调度
 */
static float Get_R_Estimate(void)
{
    float I = ina226_data.current_A;
    float V = ina226_data.voltage_V*INA226_VOLTAGE_SCALE;

    /* 电流过小时电阻计算不可信，返回参考值 */
    if (I < 0.05f) return R_REF;

    float R = V / I;
    return clampf(R, R_CLAMP_LOW, R_CLAMP_HIGH);
}
/**
 * @brief  自适应PID更新
 *
 * 增益调度策略：
 *   gain_R   = R_est / R_REF        线性调度，随负载等比缩放所有系数
 *   gain_E   = 平滑非线性函数         大误差时适度提升Kp，加快初始响应
 *
 * 仅Kp叠加误差增益；Ki/Kd只做电阻调度，避免积分饱和与微分噪声
 */
void PID_Update(float current_A)
{
    /* ---------- 误差 ---------- */
    float err = Iset - current_A;
    error = err;  /* 供OLED调试显示 */

    /* ---------- 电阻增益调度 ---------- */
    float R_est    = Get_R_Estimate();
    float gain_R   = R_est / R_REF;   /* 2Ω→0.4x，5Ω→1.0x，10Ω→2.0x */

    /* ---------- 误差非线性增益（仅作用于Kp）----------
     *
     * 采用平滑饱和函数：
     *   gain_E = 1 + (|e| / (|e| + E_BOOST_HALF)) × (E_BOOST_MAX - 1)
     *
     * |e|=0         → gain_E = 1.0  （无增强，稳态精确）
     * |e|=E_BOOST_HALF → gain_E = 1.4  （半增益点）
     * |e|→∞        → gain_E = E_BOOST_MAX = 1.8（上限，防振荡）
     */
    float abs_err  = fabsf(err);
    float gain_E   = 1.0f
                   + (abs_err / (abs_err + E_BOOST_HALF))
                   * (E_BOOST_MAX - 1.0f);

    /* ---------- 动态系数 ---------- */
    float Kp_dyn = PID_KP_BASE * gain_R * gain_E;  /* 电阻 + 误差双重调度 */
    float Ki_dyn = PID_KI_BASE * gain_R;            /* 仅电阻调度 */
    float Kd_dyn = PID_KD_BASE * gain_R;            /* 仅电阻调度 */

    /* ---------- 积分项（限幅防饱和，限幅随增益动态调整）---------- */
    float int_limit = (float)PID_DUTY_MAX / Ki_dyn;
    pid_integral   += clampf(err * PID_TS,-0.005,0.005);
    pid_integral    = clampf(pid_integral, -int_limit, int_limit);

    /* ---------- 微分项（差分 + 一阶低通滤波）---------- */
    float d_raw  = (err - prev_error) / PID_TS;
    d_filtered   = PID_D_ALPHA * d_raw
                 + (1.0f - PID_D_ALPHA) * d_filtered;
    prev_error   = err;

    /* ---------- 输出合成 ---------- */
    float output = Kp_dyn * err
                 + Kd_dyn * d_filtered;

    float new_duty = (float)pwm_duty - output - Ki_dyn * pid_integral;
    Set_PWM_Duty((uint32_t)clampf(new_duty,
                                   (float)PID_DUTY_MIN,
                                   (float)PID_DUTY_MAX));
}
static void CV_Update(float voltage_V)
{
    cverror = 18.0f - voltage_V;
    CVintegral += clampf(cverror * 0.133f,-0.005,0.005);
    CVintegral = clampf(CVintegral, -200.0f, 200.0f);
    float new_duty = (int32_t)pwm_duty - (float)(cverror * 10.0f);
    Set_PWM_Duty((uint32_t)clampf((float)new_duty- CVintegral*5,(float)PID_DUTY_MIN,(float)PID_DUTY_MAX));
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
	sprintf(msg4,"Ui:%.2fV Mode:%s",Vinadc[0] * 6 *3.3f / 4095.0f,work_mode ==MODE_CC? "CC":"CV");
	OLED_NewFrame();
	OLED_PrintString(0, 0 , msg1, &font16x16, OLED_COLOR_NORMAL);

	OLED_PrintASCIIChar(80, 0 , Isetbuffer[0]=='/0'?'0':Isetbuffer[0], &afont16x8,Ibuffercounter==0?OLED_COLOR_REVERSED:OLED_COLOR_NORMAL);
	OLED_PrintASCIIChar(95, 0 , Isetbuffer[1]=='/0'?'0':Isetbuffer[1], &afont16x8, Ibuffercounter==1?OLED_COLOR_REVERSED:OLED_COLOR_NORMAL);
	OLED_PrintASCIIChar(110, 0 , Isetbuffer[2]=='/0'?'0':Isetbuffer[2], &afont16x8, Ibuffercounter==2?OLED_COLOR_REVERSED:OLED_COLOR_NORMAL);
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

	sprintf(msg1,"Iset:%.2fA  %.2f",Iset,cverror);
	sprintf(msg2,"Io:%.3fA Uo:%.2fV",ina226_data.current_A,ina226_data.voltage_V*INA226_VOLTAGE_SCALE);
	sprintf(msg3,"compare:%d",compare);
	sprintf(msg4,"%.3f",CVintegral);
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

	//其余外设初始化
	HAL_Delay(50);

	INA226_Init(&hi2c1);

	OLED_Init();

	HAL_StatusTypeDef KBstatus;
	KBstatus = Keyboard_Init(&hi2c2);
	initIset(Isetbuffer,&Ibuffercounter);
	uint32_t last_poll_tick = 0;
	//ADC
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&Vinadc, 10);
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
		float V = ina226_data.voltage_V*INA226_VOLTAGE_SCALE;

		if (HAL_GPIO_ReadPin(IOEN_GPIO_Port, IOEN_Pin)==GPIO_PIN_SET){
			if(work_mode == MODE_CC){
				if (V >= 18.0f&&ina226_data.resistance_Ohm*Iset>=17.9){
					work_mode = MODE_CV;
					CVPID_Reset();
					continue;
				}
				PID_Update(I);
			}
			else{
				if(ina226_data.resistance_Ohm*Iset<17.9){
					work_mode = MODE_CC;
					PID_Reset();
					continue;
				}
				CV_Update(V);
			}
//			/* 判断工作模式 */
//			if (V >= 18.0f){
//				work_mode = MODE_CV;
//				/* CV模式：维持18V */
//				CV_Update(V);
//				/* CV模式下积分清零，防止切回CC时积分饱和 */
//				pid_integral = 0.0f;
//			}
//			else
//			{
//				work_mode = MODE_CC;
//				/* CC模式：PID调节电流至Iset */
//				PID_Update(I);
//			}
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
	//HAL_GPIO_TogglePin(BoardLED_GPIO_Port, BoardLED_Pin);
	HAL_Delay(20);

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
    	pwm_duty = PID_DUTY_MAX;
    	PID_Reset();
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
