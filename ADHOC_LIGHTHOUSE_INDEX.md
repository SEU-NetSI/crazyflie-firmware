# Crazyflie Lighthouse & Adhoc (DW3000) 硬件分析索引

本索引指导用户快速定位相关分析文档，解决 Lighthouse 和 Adhoc deck 之间的硬件冲突问题。

## 📋 文档导航

### 1. [快速参考指南](./ADHOC_LIGHTHOUSE_QUICK_REFERENCE.md) ⭐ 推荐先读
**用途**: 快速查询和故障排除
- 引脚分配对比表
- 三种配置模式的对比
- 常见问题 Q&A
- 快速修复方案

**适合人群**: 想快速解决问题的用户

### 2. [详细硬件分析](./ADHOC_LIGHTHOUSE_HARDWARE_ANALYSIS.md) 📚 深度解析
**用途**: 理解问题的根本原因
- Lighthouse deck 详细配置
- Adhoc deck 三种模式详解
- 资源冲突检测机制
- 故障场景详细分析
- 初始化流程时间线

**适合人群**: 想深入理解硬件设计的用户

### 3. 本文件 (你正在阅读)
**用途**: 导航和总体概览

---

## 🚨 问题症状和解决

### 症状 1: 编译/启动时输出错误
```
ERROR: Driver Gpio usage conflicts with a previously enumerated 
deck driver. No decks will be initialized!
```

**原因**: Lighthouse 和 Adhoc 默认配置的 GPIO 冲突

**快速解决**:
```bash
make menuconfig
# 在"Expansion deck configuration"中打开:
# CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y

make clean
make all
make cload
```

**详细说明**: 见快速参考中的"问题 1"部分

---

### 症状 2: DW3000 初始化失败
```
Error: DW IC is not in IDLE_RC state
Error initializing DWM3000
```

**可能原因**:
1. Adhoc deck 未被初始化（通常由配置冲突导致）
2. GPIO 初始化失败
3. 硬件复位异常

**快速解决**:
```bash
# 1. 验证配置
grep "CONFIG_DECK_ADHOC\|CONFIG_DECK_ADHOCDECK_USE_ALT_PINS" build/.config

# 2. 如果使用的是默认配置，改用 ALT_PINS
make menuconfig
# 启用: CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y

# 3. 重新编译
make clean
make all
make cload
```

**详细说明**: 见快速参考中的"问题 2"部分

---

## 📊 配置推荐

### 推荐场景 1: 同时使用 Lighthouse 和 Adhoc ✅

使用配置: **adhoc_alt_defconfig**

```bash
make adhoc_alt_defconfig
make clean
make all
make cload
```

**优点**:
- ✓ 两个 deck 都能使用
- ✓ 无资源冲突
- ✓ 硬件利用率最高

**配置文件**: `configs/adhoc_alt_defconfig`

---

### 推荐场景 2: 只使用 Adhoc

使用配置: **adhoc_defconfig**

```bash
make adhoc_defconfig
# 在 menuconfig 中禁用 CONFIG_DECK_LIGHTHOUSE
make clean
make all
make cload
```

---

### 推荐场景 3: 只使用 Lighthouse

使用默认配置并禁用 Adhoc:

```bash
make defconfig
# 在 menuconfig 中禁用 CONFIG_DECK_ADHOC
make clean
make all
make cload
```

---

## 🔍 关键概念

### Lighthouse Deck
- **用途**: GPS 卫星定位系统替代方案
- **通信方式**: UART1 (PC10, PC11)
- **资源占用**: 2 个 GPIO 引脚用于 UART

### Adhoc Deck (DW3000)
- **用途**: 超宽带 (UWB) 通讯和测距
- **SPI 通信**: 用于主控制器通信
- **中断体系**: 需要独立的 IRQ 和 RESET 引脚

### 核心问题
**Adhoc 默认配置使用 PC10/PC11 作为中断和复位引脚，而 Lighthouse 也使用相同的引脚进行 UART 通信，导致冲突**

### 解决方案
**Adhoc deck 提供多种引脚配置选项**:
- **默认**: PC10/PC11 (与 Lighthouse 冲突)
- **ALT_PINS**: PB5/PC12 (无冲突) ✅ 推荐
- **UART2_PINS**: PA2/PA3 (不与 Lighthouse 冲突，但占用 UART2)

---

## 📁 相关源代码文件

| 文件 | 作用 |
|------|------|
| `src/deck/drivers/src/lighthouse.c` | Lighthouse 驱动实现 |
| `src/deck/drivers/src/adhocdeck.c` | Adhoc/DW3000 驱动实现 |
| `src/deck/core/deck_info.c` | Deck 枚举和冲突检查 |
| `src/deck/interface/deck_core.h` | Deck 资源常数定义 |
| `src/deck/api/deck_constants.c` | GPIO 映射表 |
| `configs/adhoc_defconfig` | 默认 Adhoc 配置 |
| `configs/adhoc_alt_defconfig` | 推荐 Adhoc 配置 (使用 ALT_PINS) |

---

## 🎯 快速修复对照表

| 问题 | 症状 | 解决方案 | 预计时间 |
|------|------|--------|--------|
| GPIO 冲突 | "ERROR: Driver Gpio usage conflicts" | 启用 ALT_PINS 或禁用 Lighthouse | 5 分钟 |
| IDLE_RC 失败 | "Error: DW IC is not in IDLE_RC state" | 清理编译，使用正确的配置 | 10 分钟 |
| 两个 deck 都不工作 | 都未初始化 | 使用 ALT_PINS 配置 | 10 分钟 |
| Lighthouse 干扰 UWB | 通讯不稳定 | 检查中断冲突 | 15 分钟 |

---

## ✅ 验证清单

完成以下步骤确保配置正确:

- [ ] 查看当前的 .config 文件中的 Deck 配置
- [ ] 如果同时使用两个 Deck，确保启用了 `CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y`
- [ ] 执行 `make clean` 清理旧的编译产物
- [ ] 重新编译 `make all`
- [ ] 上传到器材 `make cload`
- [ ] 通过日志确认两个 Deck 都被正确初始化
- [ ] 测试功能是否正常

---

## 🔗 交叉参考

### 在详细硬件分析中查找:
- Lighthouse 的完整资源声明: 第 1 节
- Adhoc 三种引脚模式的对比: 第 2 节
- 资源冲突检测机制: 第 3 节
- 具体故障场景分析: 第 5 节
- DW3000 初始化流程: 第 2.3 节
- GPIO 和 EXTI 详细配置: 第 6 节
- 调试和验证步骤: 第 8 节

### 在快速参考中查找:
- 三种模式的对比表: "Adhoc Deck 三种模式"
- 配置矩阵和兼容性: "配置矩阵"
- 常见问题快速解答: "常见问题排查"
- 引脚占用汇总表: "引脚占用汇总表"
- 推荐操作流程: "推荐操作流程"

---

## 📞 故障排除流程

```
开始
  │
  ├─→ 编译时出现"ERROR: Driver Gpio usage conflicts"?
  │   YES → 启用 CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y
  │   NO  → 继续
  │
  ├─→ 启动时显示"Error: DW IC is not in IDLE_RC state"?
  │   YES → 检查是否启用了 ALT_PINS，执行 make clean
  │   NO  → 继续
  │
  ├─→ 日志中看不到 Deck 初始化消息?
  │   YES → 可能两个 Deck 都无法初始化，使用 ALT_PINS
  │   NO  → 继续
  │
  └─→ ✅ 问题解决！
```

---

## 📊 引脚冲突可视化

```
PC10/PC11 资源争夺:

场景 1: Lighthouse + Adhoc(默认)
┌────────────────────────────────┐
│ PC10 → [Lighthouse TX1]         │  冲突！
│        [Adhoc RESET]            │  ❌
├────────────────────────────────┤
│ PC11 → [Lighthouse RX1]         │  冲突！
│        [Adhoc IRQ]              │  ❌
└────────────────────────────────┘
结果: 两个 Deck 都无法使用


场景 2: Lighthouse + Adhoc(ALT_PINS)
┌────────────────────────────────┐
│ PC10 → [Lighthouse TX1]         │  独占 ✓
├────────────────────────────────┤
│ PC11 → [Lighthouse RX1]         │  独占 ✓
├────────────────────────────────┤
│ PB5  → [Adhoc IRQ via IO2]      │  独占 ✓
├────────────────────────────────┤
│ PC12 → [Adhoc RESET via IO4]    │  独占 ✓
└────────────────────────────────┘
结果: 两个 Deck 都能正常工作！ ✅
```

---

## 🎓 学习路径

### 新手用户
1. 阅读本索引文件
2. 按照"快速修复对照表"确定你的问题
3. 查看快速参考中的相应解决方案

### 中级用户
1. 阅读快速参考
2. 查看配置矩阵，选择合适的配置
3. 学习"推荐操作流程"

### 高级用户/开发者
1. 深入阅读硬件分析文档
2. 查看源代码位置表中的代码
3. 理解资源冲突检测机制
4. 考虑是否需要开发新的配置模式

---

## 💾 配置文件速查

### 预设配置命令

```bash
# 查看所有可用配置
ls -la configs/*defconfig

# 应用推荐配置
make adhoc_alt_defconfig

# 应用默认配置
make defconfig

# 应用仅 Adhoc 的配置
make adhoc_defconfig
```

### 手动创建自定义配置

```bash
make menuconfig
# 在"Expansion deck configuration"中调整选项
# Save 并 Exit
make clean
make all
make cload
```

---

## 📝 常用命令速查

```bash
# 显示当前的 Deck 配置
grep "^CONFIG_DECK_" build/.config | grep -v "# CONFIG"

# 查看详细的 Deck 启动日志
# 连接到 Crazyflie，通过 UART 监听

# 快速切换到推荐配置
make clean
make adhoc_alt_defconfig
make all
make cload

# 验证编译成功
make test

# 检查冲突（如果有冲突，编译会输出 ERROR）
make 2>&1 | grep -i "ERROR\|conflict"
```

---

## 🔐 版本信息

本分析基于 Crazyflie 固件的以下文件:
- `src/deck/drivers/src/lighthouse.c`
- `src/deck/drivers/src/adhocdeck.c` 
- `src/deck/core/deck_info.c`
- `src/deck/interface/deck_core.h`
- `src/deck/api/deck_constants.c`

**创建日期**: 2024年

---

## 📌 重要提醒

1. **建议备份**: 在修改配置前，备份你的 `.config` 文件
2. **必须 Clean**: 每次改变 Deck 配置后，必须执行 `make clean`
3. **验证上传**: 用 `make cload` 确保新二进制文件被正确上传
4. **检查日志**: 启动后通过 UART 日志验证 Deck 初始化是否成功

---

## 需要帮助？

访问以下文件获取详细信息:

| 需求 | 查看文件 | 部分 |
|------|--------|------|
| 快速查询和修复 | [快速参考](./ADHOC_LIGHTHOUSE_QUICK_REFERENCE.md) | 全部 |
| 理解根本原因 | [硬件分析](./ADHOC_LIGHTHOUSE_HARDWARE_ANALYSIS.md) | 第 3-5 节 |
| 代码位置 | [硬件分析](./ADHOC_LIGHTHOUSE_HARDWARE_ANALYSIS.md) | 第 9 节 |
| 配置推荐 | [硬件分析](./ADHOC_LIGHTHOUSE_HARDWARE_ANALYSIS.md) | 第 7 节 |

---

**最后一句话**: 如果不确定，就使用 `make adhoc_alt_defconfig` 配置，这是最安全且兼容性最好的选择。
