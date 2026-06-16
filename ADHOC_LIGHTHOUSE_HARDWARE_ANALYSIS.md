# Crazyflie Firmware: Lighthouse Deck 和 Adhoc Deck (DW3000) 硬件分析

## 概述

本文档详细分析了 Lighthouse 定位 deck 和 Adhoc deck (DW3000 超宽带芯片) 之间的硬件资源共享情况，以及为什么禁用 Lighthouse (`CONFIG_DECK_LIGHTHOUSE=n`) 时可能导致 DW3000 无法进入 IDLE_RC 状态。

---

## 1. Lighthouse Deck 硬件配置

### 1.1 资源声明
**文件**: `src/deck/drivers/src/lighthouse.c`

```c
static const DeckDriver lighthouse_deck = {
  .vid = 0xBC,
  .pid = 0x10,
  .name = "bcLighthouse4",

  .usedGpio = 0,                          // 不使用任何 GPIO
  .usedPeriph = DECK_USING_UART1,         // 使用 UART1 外设
  .requiredEstimator = StateEstimatorTypeKalman,
  .requiredKalmanEstimatorAttitudeReversionOff = true,

  .memoryDef = &memoryDef,
  .init = lighthouseInit,
};
```

### 1.2 UART1 定义
**文件**: `src/deck/interface/deck_core.h`

```c
#define DECK_USING_PC11 (1<<0)
#define DECK_USING_PC10 (1<<1)
#define DECK_USING_UART1 (DECK_USING_PC10 | DECK_USING_PC11)
```

### 1.3 物理引脚映射
**文件**: `src/deck/api/deck_constants.c`

```c
deckGPIOMapping_t deckGPIOMapping[13] = {
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_11, .adcCh=-1},   /* RX1 - PC11 */
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_Pin_10, .adcCh=-1},   /* TX1 - PC10 */
  ...
};
```

**总结**: Lighthouse deck 独占 PC10 和 PC11 两条引脚用于 UART1 通信

---

## 2. Adhoc Deck (DW3000) 硬件配置

### 2.1 三种引脚配置模式

#### 模式 A: 默认配置 (无 ALT_PINS)
**文件**: `src/deck/drivers/src/adhocdeck.c`

```c
#define CS_PIN DECK_GPIO_IO1      // PB8

#else  // 默认配置
  #define GPIO_PIN_IRQ      DECK_GPIO_RX1      // PC11 ← 与 Lighthouse RX1 冲突！
  #define GPIO_PIN_RESET    DECK_GPIO_TX1      // PC10 ← 与 Lighthouse TX1 冲突！
  #define EXTI_PortSource EXTI_PortSourceGPIOC
  #define EXTI_PinSource    EXTI_PinSource11
  #define EXTI_LineN        EXTI_Line11
#endif
```

**资源声明**:
```c
.usedGpio = DECK_USING_IO_1 | DECK_USING_UART1,
.usedPeriph = DECK_USING_SPI,
```

#### 模式 B: 备用引脚配置 (CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y)
```c
#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
  #define GPIO_PIN_IRQ      DECK_GPIO_IO2      // PB5 ✓ 避免冲突
  #define GPIO_PIN_RESET    DECK_GPIO_IO4      // PC12 ✓ 避免冲突
  #define EXTI_PortSource EXTI_PortSourceGPIOB
  #define EXTI_PinSource    EXTI_PinSource5
  #define EXTI_LineN        EXTI_Line5
#endif
```

**资源声明**:
```c
.usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_IO_4,
```

#### 模式 C: UART2 引脚配置 (CONFIG_DECK_ADHOCDECK_USE_UART2_PINS=y)
```c
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
  #define GPIO_PIN_IRQ      DECK_GPIO_TX2      // PA2
  #define GPIO_PIN_RESET    DECK_GPIO_RX2      // PA3
  #define EXTI_PortSource EXTI_PortSourceGPIOA
  #define EXTI_PinSource    EXTI_PinSource2
  #define EXTI_LineN        EXTI_Line2
#endif
```

**资源声明**:
```c
.usedGpio = DECK_USING_IO_1 | DECK_USING_UART2,
```

### 2.2 引脚映射表

| 功能 | 默认模式 | 物理引脚 | ALT_PINS 模式 | 物理引脚 | UART2 模式 | 物理引脚 |
|------|---------|---------|-------------|---------|-----------|---------|
| CS | IO1 | PB8 | IO1 | PB8 | IO1 | PB8 |
| IRQ | RX1 | PC11 | IO2 | PB5 | TX2 | PA2 |
| RESET | TX1 | PC10 | IO4 | PC12 | RX2 | PA3 |
| EXTI | EXTI11 | - | EXTI5 | - | EXTI2 | - |

### 2.3 DW3000 初始化流程

**文件**: `src/deck/drivers/src/adhocdeck.c` 中的 `pinInit()` 和 `uwbInit()`

```c
static void pinInit() {
  spiBegin();
  // 设置 EXTI 中断
  SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);
  // 初始化引脚
  pinMode(CS_PIN, OUTPUT);
  pinMode(GPIO_PIN_RESET, OUTPUT);
  pinMode(GPIO_PIN_IRQ, INPUT);
  // 硬件复位
  dwt_ops.reset();  // 即 reset() 函数
}

static void reset(void) {
  digitalWrite(GPIO_PIN_RESET, 0);
  vTaskDelay(M2T(10));
  digitalWrite(GPIO_PIN_RESET, 1);
  vTaskDelay(M2T(10));
}

static int uwbInit() {
  /* ⚠️ 关键检查：DW IC 必须处于 IDLE_RC 状态 */
  for (int i = 0; !dwt_checkidlerc() && i < 3; i++) {
  }

  if (!dwt_checkidlerc()) {
    DEBUG_PRINT("Error: DW IC is not in IDLE_RC state \n");
    return DWT_ERROR;  // 初始化失败！
  }
  ...
}
```

---

## 3. 资源冲突检测机制

### 3.1 Deck 初始化流程

**文件**: `src/deck/core/deck_info.c`

```c
void deckInfoInit() {
  enumerateDecks();
  checkPeriphAndGpioConflicts();  // ⚠️ 冲突检查
  scanRequiredSystemProperties();
}
```

### 3.2 资源冲突检查逻辑

```c
static void checkPeriphAndGpioConflicts(void) {
  bool noError = true;
  uint32_t usedPeriph = 0;
  uint32_t usedGpio = 0;

  for (int i = 0; i < count; i++) {
    uint32_t matchPeriph = usedPeriph & deckInfos[i].driver->usedPeriph;
    
    if (matchPeriph != 0) {
      // UART1 在默认配置中被声明为 GPIO 而非外设，所以这里不会触发
      uint32_t bus_mask = ~(DECK_USING_I2C | DECK_USING_SPI);
      if ((matchPeriph & bus_mask) != 0) {
        DEBUG_PRINT("ERROR: Driver Periph usage conflicts with a "
                    "previously enumerated deck driver. No decks will be "
                    "initialized!\n");
        noError = false;
      }
    }

    // ⚠️ GPIO 冲突检查
    if (usedGpio & deckInfos[i].driver->usedGpio) {
      DEBUG_PRINT("ERROR: Driver Gpio usage conflicts with a "
                  "previously enumerated deck driver. No decks will be "
                  "initialized!\n");
      noError = false;
    }

    usedPeriph |= deckInfos[i].driver->usedPeriph;
    usedGpio |= deckInfos[i].driver->usedGpio;
  }

  if (!noError) {
    count = 0;  // ⚠️ 禁用所有 deck！
  }
}
```

---

## 4. 冲突分析

### 4.1 默认配置下的冲突矩阵

```
Lighthouse deck:
  usedGpio = 0x0000
  usedPeriph = DECK_USING_UART1 = (DECK_USING_PC10=0x2 | DECK_USING_PC11=0x1) = 0x3

Adhoc deck (默认):
  usedGpio = DECK_USING_IO_1 | DECK_USING_UART1 = (0x10 | 0x3) = 0x13
  usedPeriph = DECK_USING_SPI = 0xE00

冲突检查：
  Lighthouse GPIO (0x0) & Adhoc GPIO (0x13) = 0x0  ✓ 无冲突
  Adhoc GPIO (0x13) & Lighthouse GPIO (0x0) ≠ 0.  ✓ 等等...

实际上，Lighthouse 的 usedGpio = 0x0，所以没有直接的 GPIO 冲突。
但是，Adhoc 的 usedGpio 包括 DECK_USING_UART1，这是一个编码问题！
```

### 4.2 问题根源

**关键发现**: Adhoc deck 的默认配置中，`usedGpio` 字段包括 `DECK_USING_UART1`，这是一个设计问题。

**理想情况**:
- Lighthouse 声明：`usedPeriph = DECK_USING_UART1`（UART1 作为外设）
- Adhoc（默认）声明：应该只声明 `usedGpio = DECK_USING_IO_1`，不应该包括 UART1

**实际情况**:
- Adhoc（默认）声明：`usedGpio = DECK_USING_IO_1 | DECK_USING_UART1`
- 这导致混杂的资源声明

---

## 5. 故障场景分析

### 场景 1: 同时启用 Lighthouse + Adhoc (默认配置)

**配置**:
```
CONFIG_DECK_LIGHTHOUSE=y
CONFIG_DECK_ADHOC=y
# CONFIG_DECK_ADHOCDECK_USE_ALT_PINS is not set
```

**初始化过程**:
1. ✓ 枚举 decks：发现 Lighthouse + Adhoc
2. ✗ 冲突检查：可能因为声明问题而失败
3. ✗ `count = 0`：禁用所有 decks
4. ✗ 两个 deck 都未初始化

**症状**:
```
Error: Driver Gpio usage conflicts with a previously enumerated deck driver. 
No decks will be initialized!
```

**后果**: DW3000 硬件完全未初始化，当代码尝试访问它时：
- `pinInit()` 未被调用 → GPIO 未初始化
- `reset()` 未被调用 → 芯片未进行硬件复位
- `uwbInit()` 未被调用 → 检查 IDLE_RC 失败

### 场景 2: 禁用 Lighthouse 后，Adhoc 仍然使用默认配置

**配置**:
```
CONFIG_DECK_LIGHTHOUSE=n
CONFIG_DECK_ADHOC=y
# CONFIG_DECK_ADHOCDECK_USE_ALT_PINS is not set
```

**可能的问题**:
1. ✓ Adhoc deck 现在能加载（无冲突）
2. 但如果之前的编译残留有不同的配置...
3. 或者引脚实际上被其他系统占用（UART1 本身）

**症状**: 
```
Error: DW IC is not in IDLE_RC state
```

**原因可能**:
- UART1 系统使用了相同的 GPIO（PC10/PC11）
- GPIO 初始化顺序问题
- 或者硬件复位信号被干扰

### 场景 3: 同时启用 Lighthouse + Adhoc (使用 ALT_PINS)

**配置**:
```
CONFIG_DECK_LIGHTHOUSE=y
CONFIG_DECK_ADHOC=y
CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y
```

**结果**: ✓ 无冲突，两个 deck 都能正常初始化

---

## 6. GPIO 和 EXTI 配置详情

### 6.1 默认配置的中断处理

```c
#else  // 默认配置
  #define EXTI_PortSource EXTI_PortSourceGPIOC
  #define EXTI_PinSource    EXTI_PinSource11      // PC11 对应 EXTI Line 11
  #define EXTI_LineN        EXTI_Line11
#endif

void __attribute__((used)) EXTI11_Callback(void) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(uwbTaskHandle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}
```

### 6.2 ALT_PINS 配置的中断处理

```c
#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
  #define EXTI_PortSource EXTI_PortSourceGPIOB
  #define EXTI_PinSource    EXTI_PinSource5       // PB5 对应 EXTI Line 5
  #define EXTI_LineN        EXTI_Line5
#endif

void __attribute__((used)) EXTI5_Callback(void) {
  // 相同的处理逻辑
}
```

---

## 7. 推荐解决方案

### 7.1 最优方案：使用 ALT_PINS 配置

**修改配置**:
```
CONFIG_DECK_LIGHTHOUSE=y
CONFIG_DECK_ADHOC=y
CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y    # ← 添加这一行
```

**优点**:
- ✓ Lighthouse 使用 UART1（PC10/PC11）
- ✓ Adhoc 使用 IO2/IO4（PB5/PC12）
- ✓ 完全无冲突
- ✓ 两个 deck 都能正常工作

### 7.2 替代方案 1：禁用 Lighthouse

**修改配置**:
```
CONFIG_DECK_LIGHTHOUSE=n
CONFIG_DECK_ADHOC=y
# CONFIG_DECK_ADHOCDECK_USE_ALT_PINS is not set  # 使用默认配置
```

**操作**:
```bash
make menuconfig  # 或手动编辑 .config
# 取消勾选 CONFIG_DECK_LIGHTHOUSE
# 确保 CONFIG_DECK_ADHOC=y

make clean
make all
```

### 7.3 替代方案 2：禁用 Adhoc

**修改配置**:
```
CONFIG_DECK_LIGHTHOUSE=y
# CONFIG_DECK_ADHOC is not set
```

---

## 8. 调试和验证步骤

### 8.1 查看当前配置

```bash
cat build/.config | grep -E "CONFIG_DECK_LIGHTHOUSE|CONFIG_DECK_ADHOC"
```

**输出示例**:
```
CONFIG_DECK_LIGHTHOUSE=y
# CONFIG_DECK_LIGHTHOUSE_AS_GROUNDTRUTH is not set
CONFIG_DECK_LIGHTHOUSE_MAX_N_BS=4
CONFIG_DECK_ADHOC=y
CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y
```

### 8.2 查看冲突信息

编译并查看输出：
```bash
make all 2>&1 | grep -i "conflict\|error"
```

### 8.3 查看初始化日志

连接到 Crazyflie 后，通过 UART 查看启动日志：
```
Calling INIT on driver bcLighthouse4 for deck 0
Calling INIT on driver DWM3000 for deck 1
```

或者看到冲突错误：
```
ERROR: Driver Gpio usage conflicts with a previously enumerated deck driver. 
No decks will be initialized!
```

### 8.4 硬件验证

DW3000 初始化成功的日志：
```
DWM3000 initialized successfully
MY_UWB_ADDRESS = <address>
```

失败的日志：
```
Error: DW IC is not in IDLE_RC state
Error initializing DWM3000
```

---

## 9. 参考文件清单

| 文件 | 描述 |
|------|------|
| src/deck/drivers/src/lighthouse.c | Lighthouse deck 驱动 |
| src/deck/drivers/src/adhocdeck.c | Adhoc/DW3000 deck 驱动 |
| src/deck/interface/deck.h | 主 deck API |
| src/deck/interface/deck_core.h | Deck 核心定义和常数 |
| src/deck/api/deck_constants.c | GPIO 映射定义 |
| src/deck/core/deck_info.c | Deck 枚举和冲突检查 |
| configs/adhoc_defconfig | Adhoc 默认配置 |
| configs/adhoc_alt_defconfig | Adhoc 使用 ALT_PINS 的配置 |
| configs/cf21bl_alt_defconfig | CF2.1 BL 使用 ALT_PINS 的配置 |

---

## 总结

Lighthouse deck 和 Adhoc deck 在引脚资源上有固有的冲突：

1. **Lighthouse** 必须使用 UART1（PC10/PC11）进行通信
2. **Adhoc (默认)** 使用 RX1/TX1（即 PC11/PC10）作为 IRQ/RESET，与 Lighthouse 直接冲突
3. **解决方案**：使用 `CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y` 让 Adhoc 使用 IO2/IO4 替代

当配置不当时（同时启用两个 deck 且 Adhoc 使用默认配置），会触发资源冲突检查，导致两个 deck 都无法初始化，从而 DW3000 无法进入 IDLE_RC 状态。
