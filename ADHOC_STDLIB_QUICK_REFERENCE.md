# ADHOC 模块标准库函数使用快速参考

## 汇总表

| 模块 | 文件 | 直接使用的标准库函数 | 需要 libm | 需要 _sbrk | 需要 _write |
|------|------|-------------------|---------|-----------|-----------|
| **路由** | routing.c (652行) | `rand()`, `memcpy()` | ❌ | ❌ | ❌ |
| **AODV** | aodv.c (735行) | 无 | ❌ | ❌ | ❌ |
| **OLSR** | olsr.c (789行) | `log2()`, `ceil()`, `memcpy()` | ✅ | ❌ | ❌ |
| **泛洪** | flooding.c (145行) | `rand()`, `RAND_MAX` | ❌ | ❌ | ❌ |
| **泛洪结构** | flooding_struct.c (140行) | `memset()` | ❌ | ❌ | ❌ |
| **驱动** | adhocdeck.c | 无直接 | ❌ | ❌ | ❌ |

---

## 关键发现

### ✅ 安全特性
1. **零动态内存分配** - 所有结构都是静态预分配
2. **无 malloc/free** - 不需要 _sbrk() 系统调用
3. **无格式化 I/O** - 使用自定义 DEBUG_PRINT 宏，无 printf/_write 依赖
4. **内存占用可预测** - 编译时完全确定

### ⚠️ 依赖关系
- **必须链接 libm** (用于 olsr.c 的 log2, ceil)
  ```bash
  gcc ... -lm ...  # 编译标志
  ```

### 其他特点
- **重度使用 FreeRTOS** - 所有任务/队列通过 FreeRTOS 管理
- **内存池方式** - flooding_struct.c 使用自定义预分配池
- **无错误检查** - log2/ceil 未检查输入有效性

---

## 代码统计

### 标准库函数出现次数
```
rand()     : 2 次 (routing.c, flooding.c)
memcpy()   : 2 次 (routing.c, olsr.c)
log2()     : 1 次 (olsr.c 第77行)
ceil()     : 1 次 (olsr.c 第312行)
memset()   : 1 次 (flooding_struct.c 第13行)
RAND_MAX   : 1 次 (flooding.c)
```

### 未使用的标准库函数
```
malloc, free, calloc, realloc    : ❌ (无动态分配)
printf, fprintf, sprintf         : ❌ (使用 DEBUG_PRINT)
strcpy, strcat, strlen, strcmp   : ❌ (无字符串处理)
exit, abort, assert              : ❌ (使用 FreeRTOS)
fopen, fread, fwrite             : ❌ (无文件操作)
```

---

## 编译集成指南

### 最小编译命令
```bash
# 链接 ADHOC 模块所需的库
gcc -lm \                          # 数学库（OLSR 需要）
    -I./src/deck/drivers/interface \
    -I./src/deck/drivers/src \
    -I./freertos/include \
    src/deck/drivers/src/routing.c \
    src/deck/drivers/src/aodv.c \
    src/deck/drivers/src/olsr.c \
    src/deck/drivers/src/flooding.c \
    src/deck/drivers/src/flooding_struct.c \
    src/deck/drivers/src/adhocdeck.c \
    -o adhoc.o
```

### Makefile 配置示例
```makefile
# 链接标志
LDFLAGS += -lm

# 源文件
ADHOC_SRCS = \
    src/deck/drivers/src/routing.c \
    src/deck/drivers/src/aodv.c \
    src/deck/drivers/src/olsr.c \
    src/deck/drivers/src/flooding.c \
    src/deck/drivers/src/flooding_struct.c \
    src/deck/drivers/src/adhocdeck.c
```

---

## 调试建议

### 启用调试输出
```c
// 在配置中启用这些宏：
#define ROUTING_DEBUG_ENABLE   1
#define AODV_DEBUG_ENABLE      1
#define OLSR_DEBUG_ENABLE      1
#define UWB_DEBUG_ENABLE       1
```

### 内存限制检查
```c
// 验证这些配置值
ROUTING_TABLE_SIZE_MAX          // 路由表最大条目
AODV_RREQ_BUFFER_SIZE_MAX       // AODV RREQ 缓冲区大小
NEIGHBOR_ADDRESS_MAX             // 邻居地址上限
FLOODING_TOPOLOGY_TABLE_SIZE     // 泛洪表大小
```

### 浮点精度检查
```c
// flooding.c 第53行的随机抖动
int jitter = (int) (rand() / (float) RAND_MAX * 9) - 4;
// 需要确保浮点运算精度
```

---

## 风险评估

| 风险项 | 严重性 | 缘由 | 建议 |
|--------|--------|------|------|
| OLSR log2() 输入验证 | 中 | 无错误检查 | 添加输入范围检查 |
| 静态表满溢出 | 中 | 使用 rand() 随机驱逐 | 增加表大小或实现智能驱逐 |
| 浮点精度 | 低 | jitter 计算精度 | 无关紧要，抖动用途 |
| 数学库链接缺失 | 高 | olsr.c 依赖 libm | 编译时必须加 -lm |

---

## 快速检查清单

- [x] 无 malloc/free 使用
- [x] 无 printf/_write 依赖
- [x] 无 _sbrk 依赖
- [x] 所有表大小固定
- [x] FreeRTOS 集成完整
- [x] debug 宏有默认值
- [ ] **需要链接 -lm 标志**

---

## 性能特征

```
内存占用: 完全静态分配 (编译时确定)
  ├─ 路由表:           ROUTING_TABLE_SIZE_MAX * sizeof(Route_Entry_t)
  ├─ AODV 缓冲:        AODV_RREQ_BUFFER_SIZE_MAX * sizeof(...)
  ├─ OLSR 集合:        固定大小
  └─ 泛洪表:           FLOODING_TOPOLOGY_TABLE_SIZE * sizeof(...)

CPU 开销:
  ├─ rand() 调用:      最小 (路由选择)
  ├─ memcpy 调用:      最小 (数据包转发)
  └─ math 调用:        中等 (OLSR log2/ceil - 初始化时)

延迟特征: 可预测，无动态分配延迟
```

---

**分析时间**: 2026-03-24  
**覆盖范围**: routing.c, aodv.c, olsr.c, flooding.c, flooding_struct.c, adhocdeck.c  
**总代码行数**: 2461+ 行
