/**
 ******************************************************************************
 * @file    keyboard.c
 * @brief   Emakefun 触摸矩阵键盘 V3.0 驱动实现
 *          适用于 STM32F103C8T6 HAL 库，I2C2 阻塞模式
 *
 * 硬件连接
 *   键盘 VCC → 3.3V 或 5V
 *   键盘 GND → GND
 *   键盘 SCL → PB10  (I2C2_SCL，复用开漏，建议 4.7 kΩ 上拉至 3.3V)
 *   键盘 SDA → PB11  (I2C2_SDA，复用开漏，建议 4.7 kΩ 上拉至 3.3V)
 *
 * IIC 通信协议
 *   写寄存器地址（0x01）→ 读 2 字节状态位图
 *   [START]-[0xCA W]-[0x01]-[RESTART]-[0xCB R]-[DATA_L]-[DATA_H]-[STOP]
 *   合成：key_state = DATA_L | (DATA_H << 8)
 *   bit = 1 表示该键被按下
 ******************************************************************************
 */

#include "KeyBoard.h"
#include <stdio.h>

/* ========================= 私有变量 ======================================== */

/** 内部保存的 I2C 句柄指针，由 Keyboard_Init() 绑定 */
static I2C_HandleTypeDef *s_hi2c = NULL;

/* ========================= 常量定义 ======================================== */

/** 按键字符映射表（bit 索引 → 字符） */
const char KEYBOARD_MAP[16] = {
    '1', '4', '7', '*',   /* bit  0 ~  3  Row 1 */
    '2', '5', '8', '0',   /* bit  4 ~  7  Row 2 */
    '3', '6', '9', '#',   /* bit  8 ~ 11  Row 3 */
    'A', 'B', 'C', 'D'    /* bit 12 ~ 15  Row 4 */
};

/* ========================= 函数实现 ======================================== */

/**
 * @brief  初始化键盘，绑定 I2C 句柄并检测设备是否在线
 */
HAL_StatusTypeDef Keyboard_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    s_hi2c = hi2c;

    /* 上电稳定延时 */
    HAL_Delay(100);

    /* 尝试 3 次 ACK 检测，确认键盘已就绪 */
    return HAL_I2C_IsDeviceReady(s_hi2c,
                                 KEYBOARD_I2C_ADDR,
                                 3,
                                 KEYBOARD_I2C_TIMEOUT);
}

/**
 * @brief  读取 16 位按键原始状态位图
 */
HAL_StatusTypeDef Keyboard_ReadRaw(uint16_t *key_state)
{
    HAL_StatusTypeDef ret;
    uint8_t reg     = KEYBOARD_KEY_REG;
    uint8_t data[2] = {0, 0};

    if (s_hi2c == NULL || key_state == NULL)
    {
        return HAL_ERROR;
    }

    /* Step 1: 发送寄存器地址 */
    ret = HAL_I2C_Master_Transmit(s_hi2c,
                                   KEYBOARD_I2C_ADDR,
                                   &reg,
                                   1,
                                   KEYBOARD_I2C_TIMEOUT);
    if (ret != HAL_OK)
    {
        return ret;
    }

    /* Step 2: 读取 2 字节状态数据 */
    ret = HAL_I2C_Master_Receive(s_hi2c,
                                  KEYBOARD_I2C_ADDR,
                                  data,
                                  2,
                                  KEYBOARD_I2C_TIMEOUT);
    if (ret != HAL_OK)
    {
        return ret;
    }

    /* 小端合成 16-bit 位图 */
    *key_state = (uint16_t)(data[0]) | ((uint16_t)(data[1]) << 8);
    return HAL_OK;
}

/**
 * @brief  获取当前被按下的单个按键字符
 */
char Keyboard_GetKey(void)
{
    uint16_t key_state = 0;

    if (Keyboard_ReadRaw(&key_state) != HAL_OK)
    {
        return '\0';
    }

    if (key_state == 0)
    {
        return '\0';
    }

    /* 返回位图中最低有效位对应的按键 */
    for (uint8_t i = 0; i < 16; i++)
    {
        if (key_state & (1U << i))
        {
            return KEYBOARD_MAP[i];
        }
    }

    return '\0';
}

/**
 * @brief  打印所有当前按下的按键
 */
void Keyboard_PrintAllKeys(uint16_t key_state)
{
    char    buf[48];
    uint8_t len = 0;

    for (uint8_t i = 0; i < 16; i++)
    {
        if (key_state & (1U << i))
        {
            buf[len++] = KEYBOARD_MAP[i];
            buf[len++] = ' ';
        }
    }

    if (len == 0)
    {
        return; /* 无按键，不输出 */
    }

    buf[len] = '\0';
    printf("Keys pressed: %s\r\n", buf);
}
void initIset(char* Iout,short* counter)//初始化
{
	for(int i=0;i<4;i++){Iout[i]='0';}
	Iout[3]=0;
	*counter=0;
	return;
}
void storeIset(uint16_t key,short* counter,char* Iout)//将键盘输入存入buffer
{
	short temp=*counter;
	switch(key){
		case '#':Iout[3]=1;temp=0;*counter=temp;return;
		case 'A':temp=(temp==0||(temp==2&&Iout[2]!='0'))?temp:(temp-1);Iout[temp]='0';*counter=temp;return;
		case '*':break;
		case 'B':break;
		case 'C':break;
		case 'D':break;
		default:Iout[temp]=key;temp=(temp==2)?2:(temp+1);*counter=temp;
	}
	return;
}

void getIset(char* buffer, float* Iout,short* counter)
{
	if(buffer[3]!=1)return;//当且仅当按下#后才转换
	float temp=0;short direction=0;
	for(int i=0;i<3;i++){
		if(buffer[i]!='*'){
			temp*=10;
			temp+=(buffer[i]=='/0') ? 0 : buffer[i] -'0';
		}
		else direction=i;
	}
	temp/=100;
	*Iout=temp;
	initIset(buffer,counter);
	return;
}
