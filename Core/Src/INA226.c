#include "INA226.h"
//github@Yirayi
/* ===================== 公开变量定义 ===================== */

INA226_Data            ina226_data  = {0};
volatile INA226_State  ina226_state = INA226_STATE_IDLE;

/* ===================== 私有变量 ===================== */

static I2C_HandleTypeDef *_hi2c = NULL;

/* DMA/中断接收缓冲区：必须为全局或静态，不能在栈上 */
static uint8_t _rx_buf[2];

/* ===================== 私有函数 ===================== */

static HAL_StatusTypeDef _WriteReg(uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = {
        (uint8_t)(val >> 8),
        (uint8_t)(val & 0xFF)
    };
    return HAL_I2C_Mem_Write(_hi2c, INA226_I2C_ADDR,
                              reg, I2C_MEMADD_SIZE_8BIT,
                              buf, 2, 100);
}

/* ===================== 公开函数 ===================== */

void INA226_Init(I2C_HandleTypeDef *hi2c)
{
    _hi2c        = hi2c;
    ina226_state = INA226_STATE_IDLE;
    ina226_data.data_ready = 0;

    _WriteReg(INA226_REG_CONFIG, INA226_CONFIG_VAL);
    _WriteReg(INA226_REG_CALIB,  INA226_CALIB_VAL);
    _WriteReg(INA226_REG_MASK,   INA226_MASK_VAL);

    /* 清除初始CVRF，防止初始化后立即误触发Alert */
    uint8_t tmp[2];
    HAL_I2C_Mem_Read(_hi2c, INA226_I2C_ADDR,
                      INA226_REG_MASK, I2C_MEMADD_SIZE_8BIT,
                      tmp, 2, 100);
}

void INA226_StartRead(void)
{
    /*
     * 通过ina226_state判断是否空闲
     * 外部（如EXTI回调）也可以在调用前先检查此状态
     */
    if (ina226_state != INA226_STATE_IDLE) return;

    ina226_state = INA226_STATE_RD_CURRENT;

    if (HAL_I2C_Mem_Read_IT(_hi2c, INA226_I2C_ADDR,
                              INA226_REG_CURRENT, I2C_MEMADD_SIZE_8BIT,
                              _rx_buf, 2) != HAL_OK)
    {
        /* I2C总线忙，回退到IDLE等待下次Alert */
        ina226_state = INA226_STATE_IDLE;
    }
}

void INA226_MemRxCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != _hi2c) return;

    uint16_t raw = ((uint16_t)_rx_buf[0] << 8) | _rx_buf[1];

    switch (ina226_state)
    {
        case INA226_STATE_RD_CURRENT:
            ina226_data.current_A = (int16_t)raw * INA226_CURRENT_LSB;
            ina226_state = INA226_STATE_RD_VBUS;
            HAL_I2C_Mem_Read_IT(_hi2c, INA226_I2C_ADDR,
                                  INA226_REG_VBUS, I2C_MEMADD_SIZE_8BIT,
                                  _rx_buf, 2);
            break;

        case INA226_STATE_RD_VBUS:
            ina226_data.voltage_V = (float)raw * 0.00125f;
            ina226_state = INA226_STATE_RD_MASK_CLEAR;
            /* 读Mask/Enable清除CVRF，重新arm Alert引脚 */
            HAL_I2C_Mem_Read_IT(_hi2c, INA226_I2C_ADDR,
                                  INA226_REG_MASK, I2C_MEMADD_SIZE_8BIT,
                                  _rx_buf, 2);
            break;

        case INA226_STATE_RD_MASK_CLEAR:
            if (ina226_data.current_A > 0.005f)
            {
                ina226_data.resistance_Ohm =
                    ina226_data.voltage_V / ina226_data.current_A;
            }
            else
            {
                ina226_data.resistance_Ohm = 0.0f;
            }
            ina226_data.data_ready = 1;
            /* 回到IDLE，允许下一次Alert触发新的读取 */
            ina226_state = INA226_STATE_IDLE;
            break;

        default:
            ina226_state = INA226_STATE_IDLE;
            break;
    }
}
