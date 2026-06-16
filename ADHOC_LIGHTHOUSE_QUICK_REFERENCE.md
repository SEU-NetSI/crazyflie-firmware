# Adhoc & Lighthouse Deck 快速参考

## 引脚分配对比

### Lighthouse Deck
```
┌─────────────────────────┐
│  Lighthouse Deck        │
├─────────────────────────┤
│ 硬件对接方式:           │
│ - UART1 (PC10, PC11)    │
│   └─ 无法与其他deck     │
│      共享这些引脚      │
│                         │
│ 资源占用:              │
│ usedGpio = 0x0          │
│ usedPeriph = UART1      │
└─────────────────────────┘
```

### Adhoc Deck (DW3000) - 三种模式

#### 模式 1: 默认配置 (不推荐与 Lighthouse 共存)
```
┌──────────────────────────────┐
│  Adhoc Deck (默认)           │
├──────────────────────────────┤
│ CS_PIN      PB8 (IO1)        │
│ IRQ_PIN     PC11 (RX1) ⚠️    │
│ RESET_PIN   PC10 (TX1) ⚠️    │
│ EXTI        EXTI_Line11      │
│                              │
│ 资源占用:                   │
│ usedGpio = IO1 | UART1       │
│ usedPeriph = SPI             │
│                              │
│ ⚠️ 与 Lighthouse 冲突！      │
│    操作系统 UART1 也在用     │
└──────────────────────────────┘
```

#### 模式 2: ALT_PINS (推荐)
```
┌──────────────────────────────┐
│  Adhoc Deck (ALT_PINS) ✓     │
├──────────────────────────────┤
│ CS_PIN      PB8 (IO1)        │
│ IRQ_PIN     PB5 (IO2) ✓      │
│ RESET_PIN   PC12 (IO4) ✓     │
│ EXTI        EXTI_Line5       │
│                              │
│ 资源占用:                   │
│ usedGpio = IO1 | IO2 | IO4   │
│ usedPeriph = SPI             │
│                              │
│ ✓ 无冲突！                  │
│   可与 Lighthouse 共存      │
└──────────────────────────────┘
```

#### 模式 3: UART2_PINS
```
┌──────────────────────────────┐
│  Adhoc Deck (UART2_PINS)     │
├──────────────────────────────┤
│ CS_PIN      PB8 (IO1)        │
│ IRQ_PIN     PA2 (TX2)        │
│ RESET_PIN   PA3 (RX2)        │
│ EXTI        EXTI_Line2       │
│                              │
│ 资源占用:                   │
│ usedGpio = IO1 | UART2       │
│ usedPeriph = SPI             │
│                              │
│ ✓ 不与 Lighthouse 冲突      │
│   但占用 UART2              │
└──────────────────────────────┘
```

## 配置矩阵

### ✓ 兼容的配置组合

| Config | Lighthouse | Adhoc 模式 | 状态 | 备注 |
|--------|-----------|-----------|------|------|
| A | ✓ y | 默认 | ✗ 冲突 | GPIO 资源冲突 |
| B | ✓ y | ALT_PINS | ✓ OK | 推荐配置 |
| C | ✓ y | UART2 | ✓ OK | 占用 UART2 |
| D | ✗ n | 默认 | ✓ OK | 仅使用 Adhoc |
| E | ✗ n | ALT_PINS | ✓ OK | 仅使用 Adhoc |
| F | ✓ y | ✗ n | ✓ OK | 仅使用 Lighthouse |

### 冲突分析

```
Lighthouse 使用的资源:
├─ 外设: UART1
└─ 引脚: PC10 (TX1), PC11 (RX1)

Adhoc 默认配置使用的资源:
├─ 外设: SPI
├─ GPIO: IO1 (PB8)
└─ 中断引脚:
   ├─ IRQ: PC11 (RX1) ← 与 Lighthouse 冲突！
   └─ RESET: PC10 (TX1) ← 与 Lighthouse 冲突！

❌ 冲突类型: GPIO 级别的资源冲突
   导致: 两个 deck 都无法初始化
   表现: "ERROR: Driver Gpio usage conflicts..."
```

## DW3000 初始化时间线

```
Adhoc deck 初始化顺序:

1. pinInit()
   ├─ spiBegin()
   ├─ SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource)
   ├─ pinMode(CS_PIN, OUTPUT)
   ├─ pinMode(GPIO_PIN_RESET, OUTPUT)
   ├─ pinMode(GPIO_PIN_IRQ, INPUT)
   └─ reset() [硬件复位]

2. uwbInit()
   ├─ for(3次循环) dwt_checkidlerc() ← ⚠️ 关键检查
   │                                      若失败 3 次则返回错误
   ├─ dwt_initialise(DWT_DW_INIT)
   ├─ dwt_configure(&uwbPhrConfig)
   ├─ dwt_setleds(...)
   ├─ dwt_configuretxrf(...)
   ├─ dwt_setrxantennadelay(...)
   ├─ dwt_settxantennadelay(...)
   └─ ...

❌ 如果 pinInit() 未被调用:
   - GPIO 未初始化
   - 硬件复位未执行
   - 芯片可能不在 IDLE_RC 状态

❌ 如果初始化失败:
   输出: "Error: DW IC is not in IDLE_RC state"
   返回: DWT_ERROR
```

## 常见问题排查

### 问题 1: "ERROR: Driver Gpio usage conflicts..."

**原因**: 两个 deck 使用相同 GPIO

**检查**:
```bash
grep "CONFIG_DECK_LIGHTHOUSE\|CONFIG_DECK_ADHOC\|CONFIG_DECK_ADHOCDECK_USE_ALT_PINS" build/.config
```

**解决**:
- 方案A (推荐): 启用 ALT_PINS
  ```bash
  echo "CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y" >> build/.config
  ```
- 方案B: 禁用 Lighthouse
  ```bash
  sed -i 's/CONFIG_DECK_LIGHTHOUSE=y/# CONFIG_DECK_LIGHTHOUSE is not set/' build/.config
  ```

### 问题 2: "Error: DW IC is not in IDLE_RC state"

**可能原因**:
1. ✗ Adhoc deck 未被初始化（配置冲突）
2. ✗ GPIO 初始化失败
3. ✗ 硬件复位未正确执行
4. ✗ SPI 通信异常
5. ✗ IRQ 引脚配置不匹配

**排查步骤**:

```bash
# 1. 确认编译配置
grep "CONFIG_DECK_ADHOC\|CONFIG_DECK_ADHOCDECK_USE_ALT_PINS" build/.config

# 2. 查看启动日志
# 应该看到:
# "Calling INIT on driver DWM3000 for deck..."
# "DWM3000 initialized successfully"

# 3. 清理并重新编译
make clean
make all
make cload

# 4. 如果问题仍存在，试试 ALT_PINS
make menuconfig
# 选择: Expansion deck configuration → Use alternate pins for Adhoc deck

# 5. 重新编译和上传
make clean
make all
make cload
```

### 问题 3: Lighthouse 和 Adhoc 都无法初始化

**原因**: 大概率是配置冲突

**快速修复**:

```bash
# 使用推荐配置
cat > configs/adhoc_recommended.conf << 'EOF'
CONFIG_DECK_LIGHTHOUSE=y
CONFIG_DECK_ADHOC=y
CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y
CONFIG_DECK_LIGHTHOUSE_AS_GROUNDTRUTH=y
EOF

# 应用配置
cp configs/adhoc_recommended.conf build/.config
make oldconfig
make clean
make all
```

## 引脚占用汇总表

| 设备/功能 | 引脚 | 端口 | 默认 | ALT | UART2 | 其他 |
|---------|------|------|------|-----|--------|------|
| Lighthouse Rx | PC11 | GPIOC | Lh | - | - | - |
| Lighthouse Tx | PC10 | GPIOC | Lh | - | - | - |
| Adhoc IRQ | PC11 | GPIOC | Adhoc | IO2 | TX2 | - |
| Adhoc RESET | PC10 | GPIOC | Adhoc | IO4 | RX2 | - |
| Adhoc CS | PB8 | GPIOB | Adhoc | Adhoc | Adhoc | - |
| SPI SCK | PA5 | GPIOA | SPI | SPI | SPI | - |
| SPI MISO | PA6 | GPIOA | SPI | SPI | SPI | - |
| SPI MOSI | PA7 | GPIOA | SPI | SPI | SPI | - |
| IO1 (GPIO) | PB8 | GPIOB | ✓ | ✓ | ✓ | - |
| IO2 (GPIO) | PB5 | GPIOB | - | ✓ | - | - |
| IO3 (GPIO) | PB4 | GPIOB | - | ✓ | - | - |
| IO4 (GPIO) | PC12 | GPIOC | - | ✓ | - | - |

## 代码位置导航

| 功能 | 文件位置 | 行号范围 |
|------|---------|---------|
| Lighthouse 驱动定义 | src/deck/drivers/src/lighthouse.c | 88-111 |
| Adhoc 引脚配置 | src/deck/drivers/src/adhocdeck.c | 32-62 |
| Adhoc 初始化 | src/deck/drivers/src/adhocdeck.c | 375-425 |
| 冲突检查 | src/deck/core/deck_info.c | 187-220 |
| GPIO 映射 | src/deck/api/deck_constants.c | 30-48 |
| 常数定义 | src/deck/interface/deck_core.h | 47-67 |

## 推荐操作流程

### 场景 1: 想同时使用 Lighthouse 和 Adhoc

```bash
# 编辑配置
make menuconfig

# 导航到: Expansion deck configuration
# 确保以下设置:
# ✓ bcLighthouse4
# ✓ DWM3000
# ✓ Use alternate pins for Adhoc deck (Config_DECK_ADHOCDECK_USE_ALT_PINS)

# 保存并编译
make clean
make all
make cload
```

### 场景 2: 只想使用 Adhoc Deck

```bash
# 编辑配置
make menuconfig

# 导航到: Expansion deck configuration
# 确保以下设置:
# ✗ bcLighthouse4 (禁用)
# ✓ DWM3000

# 引脚配置可选择:
# - 默认 (RX1/TX1)
# - 备用 (IO2/IO4)

make clean
make all
make cload
```

### 场景 3: 只想使用 Lighthouse

```bash
# 编辑配置
make menuconfig

# 导航到: Expansion deck configuration
# 确保以下设置:
# ✓ bcLighthouse4
# ✗ DWM3000 (禁用)

make clean
make all
make cload
```

## 关键代码片段

### 查看当前配置的状态

```bash
# 查看已启用的 deck
make menuconfig
# 在"Expansion deck configuration"中查看

# 或从 .config 文件查看
cat build/.config | grep -E "^CONFIG_DECK_" | grep -v "^# CONFIG"
```

### 快速切换配置

```bash
# 使用预设配置
make defconfig              # 使用 configs/defconfig
make adhoc_defconfig        # 使用 configs/adhoc_defconfig
make adhoc_alt_defconfig    # 使用 configs/adhoc_alt_defconfig (推荐)

# 之后
make clean
make all
make cload
```

## 总结

**一句话**: 默认情况下，Adhoc 的中断和复位引脚与 Lighthouse UART 直接冲突。解决办法是使用 `CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y`。
