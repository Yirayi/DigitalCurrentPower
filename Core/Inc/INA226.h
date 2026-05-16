#ifndef INA226_H
#define INA226_H

#include "stm32f1xx_hal.h"

/* ===================== I2C地址 ===================== */
/* A1=GND, A0=GND → 7位地址0x40，HAL要求左移一位 */
#define INA226_I2C_ADDR     (0x40 << 1)

/* ===================== 寄存器地址 ===================== */
#define INA226_REG_CONFIG   0x00
#define INA226_REG_SHUNT    0x01
#define INA226_REG_VBUS     0x02
#define INA226_REG_POWER    0x03
#define INA226_REG_CURRENT  0x04
#define INA226_REG_CALIB    0x05
#define INA226_REG_MASK     0x06
#define INA226_REG_ALERT    0x07

/* ===================== 配置值 ===================== */
/*
 * Configuration Register (0x00):
 * D15(RST)=0
 * D14-D12=010 (reserved, 保持默认)
 * D11-D9 (AVG)   = 010 → 16次平均
 * D8-D6  (VBUSCT)= 110 → 4.156ms
 * D5-D3  (VSHCT) = 110 → 4.156ms
 * D2-D0  (MODE)  = 111 → 连续测量shunt+bus
 * = 0100 0101 1011 0111 = 0x45B7
 * 总更新周期 = 4.156ms × 2通道 × 16次 ≈ 133ms
 */
#define INA226_CONFIG_VAL   0x45B7

/*
 * Calibration Register (0x05):
 * Rsample = 20mΩ, Current_LSB = 100μA/bit
 * CAL = 0.00512 / (100e-6 × 20e-3) = 2560 = 0x0A00
 * CALnew = CAL*0.98733 = 2527.6 = 0x09E0
 */
//#define INA226_CALIB_VAL    0x0A00
//#define INA226_CALIB_VAL    0x09E0
#define INA226_CALIB_VAL    0x09E5
#define INA226_CURRENT_LSB  0.0001f   /* 100μA/bit */

/*
 * Mask/Enable Register (0x06):
 * CNVR bit(D10) = 1 → 转换完成时拉低Alert引脚
 * 其余位默认0（透明模式，低电平有效）
 */
#define INA226_MASK_VAL     0x0400

/* ===================== 状态枚举（对外可见） ===================== */
/*
 * 合并busy标志和内部读取步骤为单一状态
 * 外部可通过 ina226_state == INA226_STATE_IDLE 判断是否空闲
 */
typedef enum {
    INA226_STATE_IDLE        = 0, /* 空闲，可以接受新的读取请求 */
    INA226_STATE_RD_CURRENT,      /* 正在读电流寄存器 */
    INA226_STATE_RD_VBUS,         /* 正在读电压寄存器 */
    INA226_STATE_RD_MASK_CLEAR,   /* 正在读Mask/Enable清除CVRF */
} INA226_State;

extern volatile INA226_State ina226_state;

/* ===================== 数据结构 ===================== */
typedef struct {
    float    current_A;          /* 输出电流，单位A */
    float    voltage_V;          /* 输出电压，单位V */
    float    resistance_Ohm;     /* 负载电阻，单位Ω */
    volatile uint8_t data_ready; /* 新数据就绪标志，主循环读取后清零 */
} INA226_Data;

extern INA226_Data ina226_data;

/* ===================== 函数声明 ===================== */

/**
 * @brief  初始化INA226，写入配置/校准/报警寄存器
 * @param  hi2c  I2C句柄指针，便于更换I2C总线
 */
void INA226_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  启动一次异步读取序列，由Alert引脚EXTI回调调用
 * @note   内部使用HAL_I2C_Mem_Read_IT，CPU立即返回
 */
void INA226_StartRead(void);

/**
 * @brief  I2C Mem读取完成回调的转发入口
 * @note   在HAL_I2C_MemRxCpltCallback中调用此函数
 * @param  hi2c  触发回调的I2C句柄（用于多I2C总线过滤）
 */
void INA226_MemRxCallback(I2C_HandleTypeDef *hi2c);

#endif /* INA226_H */
