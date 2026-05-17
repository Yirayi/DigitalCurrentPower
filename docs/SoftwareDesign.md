# 数控直流开关恒流电源 — 系统软件设计报告

## 一、整体软件架构

本系统以 **STM32F103C8T6** 为主控，采用**前后台架构（无 RTOS）+ 事件驱动**的混合执行模型：

| 层次 | 内容 |
|---|---|
| **应用层** | `main.c`：主循环（模式判别、PID 调用、UI 刷新） |
| **控制算法层** | `PID_Update()` / `CV_Update()`：自适应增益调度 PID |
| **外设驱动层** | `INA226.c` / `oled.c` / `KeyBoard.c` |
| **HAL 底层** | STM32 HAL：I2C、ADC+DMA、TIM、EXTI |

核心思想是**让数据采集、显存搬运全部由中断/DMA 完成，CPU 主循环专注于决策与控制**。

---

## 二、外设资源映射

| 外设 | 用途 | 关键参数 |
|---|---|---|
| **TIM1_CH1 / PA8** | 50 kHz PWM 控制 LM5175 | `Period=1439`, `Prescaler=0`（72 MHz/1440 ≈ 50 kHz） |
| **I2C1 (PB8/9) + DMA** | INA226（IT 模式接收）+ OLED（DMA 模式发送） | 100 kHz，地址 0x40 / 0x78 |
| **I2C2 (PB10/11) + DMA** | 4×4 触摸键盘（阻塞读） | 100 kHz，地址 0x65 |
| **ADC1_IN0 / PA0 + DMA** | 输入电压 Uin 采样 | 循环 DMA，10 点缓冲 |
| **EXTI PB5 (AlertIn)** | INA226 转换完成下降沿 | 优先级 1 |
| **EXTI PB15 (KeyDown)** | 启动/关闭按键上升沿 | 优先级 0 |
| **GPIO PB12 (IOEN)** | 输出级使能控制 | 初值 LOW（默认输出关闭） |

---

## 三、各模块软件设计

### 3.1 PWM 驱动模块（`tim.c`）
- 占空比通过 `__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty)` 设置，范围 `0~1439`。
- 封装函数 `Set_PWM_Duty()` 含上限钳位，是控制算法与硬件的唯一接口。
- **上电默认 `pwm_duty = PID_DUTY_MAX`**——在 LM5175 闭环极性下，最大占空比对应"输出最小"，相当于上电安全态。

### 3.2 INA226 测量模块（`INA226.c` + `INA226.h`）—— 异步状态机
最具特色的模块。**完全异步、无阻塞**：

**配置**：16 次平均 × (4.156 ms × 2 通道) ≈ **133 ms 一次更新**；Mask 寄存器开启 CNVR，转换完成后 Alert 引脚被拉低。

**状态机**（`INA226_State`）：

```
IDLE ─Alert中断─► RD_CURRENT ─I2C回调─► RD_VBUS ─回调─► RD_MASK_CLEAR ─回调─► IDLE
                       ↑                                                      │
                       └──────────────  data_ready=1 通知主循环  ──────────────┘
```

- `INA226_StartRead()` 用 `HAL_I2C_Mem_Read_IT` 启动，CPU 立刻返回；
- `INA226_MemRxCallback()` 由 HAL I2C 中断回调驱动，按状态读下一寄存器；
- 第三步读 Mask 寄存器**专门用于清 CVRF**，否则 Alert 不会重新 arm；
- 读完后计算 `R_load = V/I`（电流 > 5 mA 才算，避免除零），置 `data_ready` 给主循环。

### 3.3 OLED 模块（`oled.c`）—— 帧缓冲 + DMA
- 8×128 显存数组 `OLED_GRAM[8][128]`；
- 三段式调用：`OLED_NewFrame()` → `OLED_PrintString()` 系列写显存 → `OLED_ShowFrame()` 用 **DMA** 一次性吐出 1025 字节；
- `oled_dma_busy` 标志在 `HAL_I2C_MasterTxCpltCallback` 中清零，防止前一帧未发完就启动下一帧；
- 初始化最后一帧用 `OLED_ShowFrame_Blocking`，保证 `0xAF` 点屏命令在显存清零之后才发出。

### 3.4 键盘模块（`KeyBoard.c`）—— I2C 位图 + 数字缓冲区
**底层读取**：`Keyboard_ReadRaw()` 写寄存器地址 `0x01` → 读 2 字节 → 拼成 16-bit 位图，bit=1 表示对应键按下，再通过 `KEYBOARD_MAP[16]` 映射为字符。

**输入逻辑**（关键设计）：
- `Isetbuffer[4]`：3 字节数字 + 1 字节"已确认"标志位；
- `storeIset()`：数字键写入当前位置；`A` 键退格；`#` 键置 `buffer[3]=1` 触发确认；
- `getIset()`：仅当 `buffer[3]==1` 时把 `XYZ → X.YZ` 转换为 `Iset`，再重置 buffer；
- **去抖**：主循环用 `key != keybuffer` 判断状态变化，**一次按下只取一次值**。

### 3.5 输入电压采样（`adc.c`）
- ADC1_IN0 单通道，41.5 周期采样；
- DMA1_Channel1 **循环模式** 持续把转换结果灌入 `Vinadc[10]`，CPU 不参与；
- 主循环直接读 `Vinadc[0] × 6 × 3.3 / 4095`（× 6 还原 1:6 分压比）。

### 3.6 控制算法模块（`main.c` 核心）

#### 3.6.1 CC 模式 —— **双重增益调度自适应 PID**
基础参数：`Kp₀=50, Ki₀=20, Kd₀=5, Ts=0.133 s`（**与 INA226 采样周期严格匹配**）。

**第一重：电阻增益调度**

```
gain_R = R_est / R_REF       (R_REF = 5Ω, R_est 钳位在 [2, 10])
```

- 1 Ω 重载 → 增益小，避免过冲；10 Ω 轻载 → 增益大，加快响应。
- 同时作用于 Kp/Ki/Kd，使闭环带宽**对负载变化保持基本恒定**。

**第二重：误差非线性增益（仅作用于 Kp）**

```
gain_E = 1 + |e| / (|e|+0.15) × (1.8 - 1)
```

- 平滑饱和函数，`|e|→0` 时 gain_E=1（保稳态精度），`|e|→∞` 时趋近 1.8（防振荡）；
- 类似"专家 PID"，大误差时加猛劲，小误差时温柔。

**积分/微分稳健性**：
- 积分单步增量限幅 `±0.005`（防突变），积分总值动态限幅 `±DUTY_MAX/Ki_dyn`；
- 微分项一阶低通 `α=0.3`，对 INA226 噪声鲁棒；
- 输出方程：`new_duty = pwm_duty - (Kp·e + Kd·d') - Ki·∫e`（增量式，注意符号——LM5175 在该拓扑下占空比与电流呈反向关系）。

#### 3.6.2 CV 模式 —— 简化 PI

```
cverror = 18 - V
new_duty = pwm_duty - 10·cverror - 5·CVintegral
```

保护输出不超过 18V，结构精简、调试方便。

#### 3.6.3 模式切换逻辑（`main.c:362`）

```
CC → CV : V ≥ 18 且 R·Iset ≥ 17.9
CV → CC : R·Iset < 17.9
```

**双重判据避免抖动**：仅靠 `V≥18` 判定会反复跳变；引入 `R·Iset`（即按当前电阻全功率输出时的理论电压）作为"预测电压"，**只有真的拉不下来才切到 CV**，切回时同样以该预测值为依据，并重置对应模式积分项。

### 3.7 启动/停止控制（`main.c:472` EXTI 回调）
按下 PB15 → 翻转 `IOEN` 引脚电平 → 同时 `PID_Reset()` 并把 `pwm_duty` 拉回 `DUTY_MAX`（安全态），下次启动从最小输出开始软启动。

---

## 四、模块协同工作分析

### 4.1 数据流总览

```
                ┌────── DMA 循环 ──────────► Vinadc[]
   Uin (PA0) ──►│ ADC1

   INA226 ──┐ ┌─Alert(EXTI PB5)─► StartRead ─┐
            └►│  ┌──I2C1 IT 中断回调状态机───┘                  ┌──► OLED 显示
              │  └─ data_ready ─► 主循环 ─► PID/CV ─► PWM ─► LM5175

   键盘 ──I2C2 轮询──► Iset 缓冲 ─#确认─► Iset

   启动键 ─EXTI(PB15)─► IOEN 翻转 + PID Reset

   OLED ◄── I2C1 DMA TX ── OLED_ShowFrame (主循环 20ms)
```

### 4.2 三种节拍并存

| 节拍 | 周期 | 触发方式 | 工作内容 |
|---|---|---|---|
| **控制环** | ~133 ms | INA226 Alert 中断 | 真正的 PID 闭环更新 |
| **UI/键盘** | 20 ms | 主循环 `HAL_Delay` | 键盘轮询 + OLED 刷新 |
| **Uin 采样** | 连续 | DMA 后台 | 永远准备最新值 |
| **启停** | 异步 | EXTI 中断 | 即时响应 |

**关键巧妙之处**：PID 采样周期 Ts=0.133 s **不是凭空设的**，而是 INA226 的硬件实际更新周期。这样保证每次 PID 计算都用到一帧新数据，**没有"假更新"**，差分项不会因重复采样而失真。

### 4.3 共享资源（I2C1）冲突回避

I2C1 同时挂着 INA226（读）和 OLED（写）。两者都是异步操作，必须协调：

1. **INA226 侧**：用 `ina226_state` 表征忙闲，主循环每次只在 IDLE 时才发起新读取；
2. **OLED 侧**：用 `oled_dma_busy` 标志，主循环每次在 `OLED_ShowFrame()` 入口判断，正忙时**直接跳过当帧**；
3. **时序天然错峰**：INA226 读取序列约 1~2 ms（3 次 2 字节），OLED DMA 约 100 ms 量级（1025 字节 @100 kHz ≈ 100 ms），但 OLED 帧间隔有 20 ms，主循环执行体内 INA226 检查在 OLED 发起之前——理论上仍存在一定碰撞窗口，HAL I2C 的 `BUSY` 状态会让后发起的 `_IT/_DMA` 返回错误，本设计中通过 `ina226_state` 回滚到 IDLE 来等待下一次 Alert，从而**自然容错**。

### 4.4 中断/优先级设计

| 中断 | 优先级 | 作用 |
|---|---|---|
| `I2C1_EV` / `ADC1_2` / `EXTI15_10` (KeyDown) | **0**（最高） | 数据完整性 + 启停响应 |
| `EXTI9_5` (AlertIn) | 1 | 启动 INA226 读取序列 |

启动按键给到最高级，确保**任何时候都能立即关断输出**，是安全设计。

### 4.5 主循环执行流（`main.c:346`）

```c
while(1) {
    /* 1) 数据采集触发与处理 */
    if (ina226_state == IDLE) INA226_StartRead();
    if (ina226_data.data_ready) {
        data_ready = 0;
        if (IOEN == ON) {
            根据 V 和 R·Iset 决定 CC/CV 模式
            CC: PID_Update(I)  /  CV: CV_Update(V)
        }
    }

    /* 2) 键盘轮询（20 ms 间隔） */
    if (tick 到 20ms) {
        key = Keyboard_GetKey();
        if (key 变化 && key 非空) storeIset(...);
    }
    getIset(...);   // 仅在 # 确认后更新 Iset

    /* 3) UI 刷新 */
    OledRefreshData();
    OLED_ShowFrame();

    HAL_Delay(20);
}
```

> 注：`getIset` 每轮都调用，但内部以 `buffer[3]==1` 为门，**未按 # 时不会更新 Iset**——这是个轻量的"事件式"消费模式。

### 4.6 系统启动流程

```
HAL_Init → SystemClock(72M) → MX_xxx_Init (GPIO/DMA/I2C/TIM/ADC)
   → HAL_TIM_PWM_Start (占空比 = 80% × MAX，安全偏置)
   → HAL_Delay(50)  // 等待外设上电稳定
   → INA226_Init → OLED_Init → Keyboard_Init → initIset
   → ADC 校准 → HAL_ADC_Start_DMA
   → 主循环
```

外设初始化顺序是有讲究的：**先 PWM 安全态、再等待 50 ms、再初始化 I2C 从机**，避免 INA226/OLED 上电不稳被误访问。

---

## 五、设计亮点

1. **完全非阻塞 I/O**：INA226 中断状态机 + OLED DMA + ADC 循环 DMA，主循环不被任何外设拖慢，PID 实时性高。
2. **PID 采样周期严格对齐硬件**：`Ts = 0.133 s` 来自 INA226 的实际转换节拍，避免过采样引入的微分噪声。
3. **双重自适应增益调度**：电阻调度（线性）+ 误差调度（非线性饱和），用一组系数稳定覆盖 1Ω~10Ω、0~2A 全工况。
4. **CC/CV 模式切换的预测判据** `R·Iset`：避免临界条件下的模式抖动。
5. **键盘"缓冲 + 确认"输入模式**：所见即所得地展示输入过程，#键提交，避免误触。
6. **上电默认安全态**：`IOEN=0` 且 `pwm_duty=MAX`（在该拓扑下对应零输出），按键启动后才软启动。
7. **关断瞬时复位**：按下停止键的同时 `PID_Reset()`，下次启动不会带着旧的积分量"暴冲"。

---

## 六、可优化方向（供参考）

| 现状 | 改进 |
|---|---|
| INA226 平均 16 次，更新 133 ms 偏慢 | 减到 4 次约 33 ms，提升控制带宽 |
| CV 模式 PI 系数硬编码 `10, 5` | 同样做电阻调度，与 CC 一致 |
| OLED 每 20 ms 重绘 + 浮点 `sprintf` | 调到 50~100 ms，或仅刷新变化字段 |
| 键盘仅靠状态变化去抖 | 增加 ≥2 帧一致性确认 + 长按重复 |
| `HAL_Delay(20)` 阻塞主循环 | 改为 `HAL_GetTick()` 非阻塞调度，便于将来加协议栈/串口 |
| Iset 输入无范围校验 | `getIset` 末尾 `clamp(0, 2.0)` |
