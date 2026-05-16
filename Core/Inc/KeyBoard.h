/**
 ******************************************************************************
 * @file    keyboard.h
 * @brief   Emakefun 触摸矩阵键盘 V3.0 驱动头文件
 *          适用于 STM32F103C8T6 HAL 库，I2C2 阻塞模式
 ******************************************************************************
 */

#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ========================= 宏定义 ========================================= */

/** 键盘 I2C 7-bit 地址，HAL 调用时自动左移 1 位 */
#define KEYBOARD_I2C_ADDR       (0x65 << 1)

/** 读取按键状态的寄存器地址 */
#define KEYBOARD_KEY_REG        0x01U

/** I2C 单次操作超时（ms） */
#define KEYBOARD_I2C_TIMEOUT    10U

/** 主循环建议的最小轮询间隔（ms） */
#define KEYBOARD_POLL_INTERVAL  20U

/**
 * @brief 16 个按键的字符映射（按 bit 索引顺序）
 *
 *  Bit  0 → '1'    Bit  1 → '2'    Bit  2 → '3'    Bit  3 → 'A'
 *  Bit  4 → '4'    Bit  5 → '5'    Bit  6 → '6'    Bit  7 → 'B'
 *  Bit  8 → '7'    Bit  9 → '8'    Bit 10 → '9'    Bit 11 → 'C'
 *  Bit 12 → '*'    Bit 13 → '0'    Bit 14 → '#'    Bit 15 → 'D'
 */
extern const char KEYBOARD_MAP[16];

/* ========================= 函数声明 ======================================== */

/**
 * @brief  初始化键盘，绑定 I2C 句柄并检测设备是否在线
 * @param  hi2c  指向 HAL I2C 句柄的指针（传入 &hi2c2）
 * @retval HAL_OK      初始化成功，设备已就绪
 *         HAL_ERROR   设备未响应，请检查接线与地址
 */
HAL_StatusTypeDef Keyboard_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  读取 16 位按键原始状态位图
 * @param  key_state  输出参数；bit = 1 表示对应按键被按下
 * @retval HAL_OK    读取成功
 *         其他值   I2C 通信失败
 */
HAL_StatusTypeDef Keyboard_ReadRaw(uint16_t *key_state);

/**
 * @brief  获取当前被按下的单个按键字符
 *         若同时有多键按下，返回位图中最低有效位对应的键
 * @retval 按键字符（'0'~'9', 'A'~'D', '*', '#'）
 *         '\0'  无按键或通信失败
 */
char Keyboard_GetKey(void);

/**
 * @brief  将 16 位位图中所有被按下的键通过 printf 打印出来
 * @param  key_state  由 Keyboard_ReadRaw() 获取的位图值
 * @note   需要先完成 printf 重定向（__io_putchar → UART）
 */
void Keyboard_PrintAllKeys(uint16_t key_state);

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_H */
