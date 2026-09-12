# MermanLogger 使用手册

## 一、概述

MermanLogger 是一个基于 `spdlog` 的 **Header-only** 异步/同步日志框架，专为本项目各模块提供统一、高性能、低耦合的日志能力。

**核心特性：**
- ✅ **Release / Debug 双模式**：运行时通过 `--debug true` 或配置切换。
- ✅ **免手写模块名**：模块名由 CMake `PROJECT_NAME` 自动注入编译期宏，无需手动传递。
- ✅ **模块日志文件隔离**：各模块日志独立隔离输出（如 `merman_control.log`、`merman_slam.log`）。
- ✅ **滚动备份与追加模式**：每个模块最大 50MB × 5 份滚动保护，重启后日志自动追加（不刷新/清空）。
- ✅ **动态线程 ID 打印开关**：可通过 `logger.yaml` 中的 `print_tid` 控制。
- ✅ **全局毫秒高精度支持**：Release 和 Debug 版本的日志时间戳均精确至毫秒。
- ✅ **布尔文本化输出**：布尔值（包括流式 stream 打印）均自动格式化为明文 `"true"` / `"false"`，不再输出 `1` / `0`。

---

## 二、配置说明 (`logger.yaml`)

配置文件位于 `merman_common/config/logger.yaml`。框架在调用 `Initialize` 时会自动读取并轻量化无依赖解析：

```yaml
logger:
  debug: false     # 是否开启 Debug 调试级日志，默认 false (Release 简洁模式)
  enable: true     # 日志全局开关，若为 false 则关闭全部日志输出
  async: true      # 异步开关，若为 false 则不启动异步线程，切换为同步直写模式
  print_tid: true  # 线程ID开关，控制是否在日志中打印 [tid:xxxxxx]
  log_dir: "../logs" # 日志文件输出目录
```

---

## 三、日志输出格式约定

为防止独立模块日志中存在冗余前缀，**Debug 和 Release 两档输出均已剔除模块名（如 `merman_slam`）**，其格式如下：

### 1. Release 模式 (`debug: false` 且 `print_tid: true`)
> `[时间.毫秒] [tid:线程ID] [日志级别] [函数名] 消息内容`
```text
[2026-07-16 11:44:18.144] [tid:749624] [info] [get_slam_version] SLAM version queried: v0.1
```
*(若关闭 `print_tid`，则会自动变为：`[2026-07-16 11:44:30.575] [info] [get_slam_version] ...`)*

### 2. Debug 模式 (`debug: true` 且 `print_tid: true`)
> `[时间.毫秒] [tid:线程ID] [文件名:行号] [日志级别] [函数名] 消息内容`
```text
[2026-07-16 11:37:34.515] [tid:723219] [merman_slam_interface.cpp:13] [info] [get_slam_version] --- SLAM Version queried: v0.1 ---
```

---

## 四、数据类型格式化规范

### 1. 浮点数精确控制
不需要使用额外的宏，直接在格式化占位符中指定小数位数即可：
```cpp
double pi_val = 3.14159265;
ML_INFO("Formatting Float (2 decimals): {:.2f}", pi_val); // -> 3.14
ML_INFO("Formatting Float (3 decimals): {:.3f}", pi_val); // -> 3.142
```

### 2. 布尔型输出
无论是标准格式化还是 `_STREAM` 流式日志，布尔类型都会输出明文 `true`/`false`：
```cpp
bool is_active = true;
ML_INFO("Active: {}", is_active); // -> Active: true
ML_INFO_STREAM << "Active status: " << is_active; // -> Active status: true
```

---

## 五、API 宏使用规范

| 宏定义 | 组别 | 输出时机 | 备注 |
|---|---|---|---|
| `ML_INFO(fmt, ...)` | Release | 始终输出 | 普通信息 |
| `ML_WARNING(fmt, ...)` | Release | 始终输出 | 警告信息 |
| `ML_ERROR(fmt, ...)` | Release | 始终输出 | 错误信息 |
| `ML_INFO_DEBUG(fmt, ...)` | Debug | 仅在 debug 开关开启时输出 | 调试信息 |
| `ML_WARNING_DEBUG(fmt, ...)`| Debug | 仅在 debug 开关开启时输出 | 调试警告 |
| `ML_ERROR_DEBUG(fmt, ...)` | Debug | 仅在 debug 开关开启时输出 | 调试错误 |
| `ML_INFO_STREAM` | Stream | 始终输出（Release 级别） | 支持 `<<` 拼接 |
| `ML_WARNING_STREAM` | Stream | 始终输出（Release 级别） | 支持 `<<` 拼接 |
| `ML_ERROR_STREAM` | Stream | 始终输出（Release 级别） | 支持 `<<` 拼接 |

---

## 六、如何在新包/新模块中接入

### 1. 修改模块 `CMakeLists.txt`
在目标库或可执行文件的构建配置中注入 `MERMAN_MODULE_NAME` 编译宏，让日志系统自动识别并分流到独立的日志文件（例如 `merman_slam.log`）：
```cmake
target_compile_definitions(你的模块目标名 PRIVATE
    MERMAN_MODULE_NAME="${PROJECT_NAME}"
)
```

### 2. 在业务代码中引入头文件
```cpp
#include "merman_logger/merman_logger.hpp"
```

### 3. 全局初始化仅在程序入口 `main` 中执行一次
在主入口 `merman_robot.cpp` 中解析命令行 `--debug` 参数并初始化：
```cpp
int main(int argc, char *argv[]) {
    // 解析 --debug 参数以决定是否覆盖配置文件中的 debug 标志
    bool debug_mode = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--debug" && i + 1 < argc) {
            debug_mode = (std::string(argv[i + 1]) == "true");
        }
    }
    
    // 初始化日志存放于 "../logs" (即 merman_common/logs)
    MermanLogger::GetInstance().Initialize("../logs", debug_mode);
    
    // ... 正常业务启动 ...
}
```

---

## 七、测试示例

在 `merman_slam` 模块的版本获取函数中的应用实践：
```cpp
#include "merman_logger/merman_logger.hpp"

std::string MermanSlamInterface::get_slam_version() {
    ML_INFO("--- SLAM Version queried: v0.1 ---");
    
    double pi_val = 3.14159265;
    ML_INFO("Float formatting test: {:.3f}", pi_val);
    
    bool is_active = true;
    ML_INFO("Boolean formatting test: {}", is_active);
    
    return "v0.1";
}
```
编译通过运行后，会在 `merman_common/logs/merman_slam.log` 中生成对应的持久化文件。