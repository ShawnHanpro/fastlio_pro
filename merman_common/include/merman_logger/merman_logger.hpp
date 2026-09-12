#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

// ============================================================
// bool 类型 fmt 格式化器：输出 "true" / "false" 而非 1 / 0
// ============================================================
template <>
struct fmt::formatter<bool> : fmt::formatter<std::string_view>
{
    auto format(bool val, fmt::format_context& ctx) const
    {
        return fmt::formatter<std::string_view>::format(
            val ? "true" : "false", ctx);
    }
};

// ============================================================
// 浮点数辅助宏：默认保留 3 位小数
// 用法: ML_INFO("speed: {}", ML_FLOAT(vel));
// ============================================================
#define ML_FLOAT(val) fmt::format("{:.3f}", static_cast<double>(val))

// ============================================================
// 流式日志辅助类
// ============================================================
class MermanLogStreamHelper
{
public:
    MermanLogStreamHelper(std::shared_ptr<spdlog::logger> logger,
                          spdlog::level::level_enum       level,
                          spdlog::source_loc              loc)
        : logger_(logger), level_(level), loc_(loc)
    {
        stream_ << std::boolalpha;
    }

    ~MermanLogStreamHelper()
    {
        if (logger_ && logger_->should_log(level_))
        {
            logger_->log(loc_, level_, "[{}] {}",
                         loc_.funcname, stream_.str());
        }
    }

    template <typename T>
    MermanLogStreamHelper& operator<<(const T& val)
    {
        if (logger_ && logger_->should_log(level_))
        {
            stream_ << val;
        }
        return *this;
    }

private:
    std::shared_ptr<spdlog::logger> logger_;
    spdlog::level::level_enum       level_;
    spdlog::source_loc              loc_;
    std::ostringstream              stream_;
};

// ============================================================
// 日志管理器单例
// ============================================================
class MermanLogger
{
public:
    static MermanLogger *GetInstance()
    {
        static MermanLogger instance;
        return &instance;
    }

    static std::string get_logger_version() {
        return "v.26.07.16";
    }

    /**
     * @brief 初始化日志系统 (全局仅需调用一次，通常在 main 中调用)
     * @param log_dir  日志文件存放目录，默认 "../logs"
     * @param debug    是否开启 debug 模式
     */
    inline void Initialize(const std::string& log_dir = "../logs",
                           bool               debug   = false)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_)
        {
            return;
        }

        bool enable_log = true;
        bool async_log  = true;
        bool debug_mode = debug;
        bool print_tid  = true;
        std::string final_log_dir = log_dir;

        // 手动解析 logger.yaml，避免对 yaml-cpp 产生依赖
        std::ifstream ifs("../config/logger.yaml");
        if (ifs.is_open())
        {
            std::string line;
            while (std::getline(ifs, line))
            {
                std::string clean;
                for (char c : line)
                {
                    if (c != ' ' && c != '\t' && c != '\r' && c != '\n')
                    {
                        clean += c;
                    }
                }
                if (clean.empty() || clean[0] == '#') continue;

                auto pos = clean.find(':');
                if (pos != std::string::npos)
                {
                    std::string key = clean.substr(0, pos);
                    std::string val = clean.substr(pos + 1);

                    if (key == "debug")
                    {
                        if (!debug)
                        {
                            debug_mode = (val == "true");
                        }
                    }
                    else if (key == "enable")
                    {
                        enable_log = (val == "true");
                    }
                    else if (key == "async")
                    {
                        async_log = (val == "true");
                    }
                    else if (key == "print_tid")
                    {
                        print_tid = (val == "true");
                    }
                    else if (key == "log_dir")
                    {
                        if (val.size() >= 2 && (val.front() == '"' || val.front() == '\''))
                        {
                            val = val.substr(1, val.size() - 2);
                        }
                        final_log_dir = val;
                    }
                }
            }
            ifs.close();
        }

        try
        {
            log_dir_     = final_log_dir;
            debug_mode_  = debug_mode;
            enable_log_  = enable_log;
            async_log_   = async_log;
            print_tid_   = print_tid;

            // 1. 异步线程池
            spdlog::init_thread_pool(8192, 1);

            // 2. 控制台 Sink
            console_sink_ =
                std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink_->set_level(spdlog::level::trace);

            // 3. 根据 print_tid_ 动态设置日志格式 (已去除模块名并包含毫秒)
            std::string pattern;
            if (debug_mode_)
            {
                if (print_tid_)
                {
                    pattern = "[%Y-%m-%d %H:%M:%S.%e] [tid:%t] [%s:%#] [%^%l%$] %v";
                }
                else
                {
                    pattern = "[%Y-%m-%d %H:%M:%S.%e] [%s:%#] [%^%l%$] %v";
                }
            }
            else
            {
                if (print_tid_)
                {
                    pattern = "[%Y-%m-%d %H:%M:%S.%e] [tid:%t] [%^%l%$] %v";
                }
                else
                {
                    pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";
                }
            }
            spdlog::set_pattern(pattern);

            // 4. WARN 及以上立即刷盘
            spdlog::flush_on(spdlog::level::warn);

            // 5. 定时 3s 刷盘
            spdlog::flush_every(std::chrono::seconds(3));

            initialized_ = true;
        }
        catch (const spdlog::spdlog_ex& ex)
        {
            std::cerr << "MermanLogger init failed: "
                      << ex.what() << std::endl;
        }
    }

    /** @brief 运行时查询当前是否处于 debug 模式 */
    inline bool IsDebugMode() const { return debug_mode_; }

    /**
     * @brief 获取 / 创建模块专属 Logger
     *
     * 每个模块拥有独立的滚动日志文件 (50 MB × 5 份)
     */
    inline std::shared_ptr<spdlog::logger> GetLogger(
        const std::string& name)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_)
        {
            mutex_.unlock();
            Initialize();
            mutex_.lock();
        }

        auto it = loggers_.find(name);
        if (it != loggers_.end())
        {
            return it->second;
        }

        // 每个模块独立文件 Sink: 50 MB, 保留 5 份
        auto file_path = log_dir_ + "/" + name + ".log";
        auto file_sink =
            std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                file_path, 50 * 1024 * 1024, 5);
        file_sink->set_level(spdlog::level::debug);

        std::vector<spdlog::sink_ptr> sinks = {console_sink_,
                                                file_sink};

        std::shared_ptr<spdlog::logger> logger;
        if (async_log_)
        {
            logger = std::make_shared<spdlog::async_logger>(
                name, sinks.begin(), sinks.end(),
                spdlog::thread_pool(),
                spdlog::async_overflow_policy::overrun_oldest);
        }
        else
        {
            logger = std::make_shared<spdlog::logger>(
                name, sinks.begin(), sinks.end());
        }

        // 显式为 Logger 设置格式 (已去除模块名，支持毫秒与动态线程 ID 开关)
        std::string pattern;
        if (debug_mode_)
        {
            if (print_tid_)
            {
                pattern = "[%Y-%m-%d %H:%M:%S.%e] [tid:%t] [%s:%#] [%^%l%$] %v";
            }
            else
            {
                pattern = "[%Y-%m-%d %H:%M:%S.%e] [%s:%#] [%^%l%$] %v";
            }
        }
        else
        {
            if (print_tid_)
            {
                pattern = "[%Y-%m-%d %H:%M:%S.%e] [tid:%t] [%^%l%$] %v";
            }
            else
            {
                pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";
            }
        }
        logger->set_pattern(pattern);

        if (enable_log_)
        {
            logger->set_level(spdlog::level::trace);
        }
        else
        {
            logger->set_level(spdlog::level::off);
        }
        spdlog::register_logger(logger);
        loggers_[name] = logger;
        return logger;
    }

    /** @brief 全局日志资源优雅释放 */
    inline void Shutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        spdlog::shutdown();
        loggers_.clear();
        console_sink_.reset();
        log_dir_.clear();
        initialized_ = false;
    }

private:
    MermanLogger()  = default;
    ~MermanLogger() { Shutdown(); }

    std::mutex mutex_;
    std::unordered_map<std::string,
                       std::shared_ptr<spdlog::logger>>
                       loggers_;
    spdlog::sink_ptr   console_sink_;
    std::string        log_dir_;
    bool               initialized_ = false;
    bool               debug_mode_  = false;
    bool               enable_log_  = true;
    bool               async_log_   = true;
    bool               print_tid_   = true;
};

// ============================================================
// CMake 自动注入的模块名兜底
// ============================================================
#ifndef MERMAN_MODULE_NAME
#define MERMAN_MODULE_NAME "unknown_module"
#endif

// ============================================================
// 内部宏：格式化调用
// ============================================================
#define MERMAN_LOGGER_CALL_(logger, lvl, fmt_str, ...)         \
    if (logger && logger->should_log(lvl))                     \
    {                                                          \
        logger->log(                                           \
            spdlog::source_loc{__FILE__, __LINE__,             \
                               __FUNCTION__},                  \
            lvl, "[{}] " fmt_str,                              \
            __FUNCTION__, ##__VA_ARGS__);                       \
    }

// ============================================================
// Release 组宏 —— 始终可用
// ============================================================
#define ML_INFO(fmt_str, ...)                                  \
    MERMAN_LOGGER_CALL_(                                       \
        MermanLogger::GetInstance()->GetLogger(           \
            MERMAN_MODULE_NAME),                               \
        spdlog::level::info, fmt_str, ##__VA_ARGS__)

#define ML_WARNING(fmt_str, ...)                               \
    MERMAN_LOGGER_CALL_(                                       \
        MermanLogger::GetInstance()->GetLogger(           \
            MERMAN_MODULE_NAME),                               \
        spdlog::level::warn, fmt_str, ##__VA_ARGS__)

#define ML_ERROR(fmt_str, ...)                                 \
    MERMAN_LOGGER_CALL_(                                       \
        MermanLogger::GetInstance()->GetLogger(           \
            MERMAN_MODULE_NAME),                               \
        spdlog::level::err, fmt_str, ##__VA_ARGS__)

// ============================================================
// Debug 组宏 —— 仅在 debug 模式下输出
// ============================================================
#define ML_INFO_DEBUG(fmt_str, ...)                             \
    if (MermanLogger::GetInstance().IsDebugMode())       \
    {                                                          \
        MERMAN_LOGGER_CALL_(                                   \
            MermanLogger::GetInstance()->GetLogger(       \
                MERMAN_MODULE_NAME),                           \
            spdlog::level::info, fmt_str, ##__VA_ARGS__);      \
    }

#define ML_WARNING_DEBUG(fmt_str, ...)                          \
    if (MermanLogger::GetInstance().IsDebugMode())       \
    {                                                          \
        MERMAN_LOGGER_CALL_(                                   \
            MermanLogger::GetInstance()->GetLogger(       \
                MERMAN_MODULE_NAME),                           \
            spdlog::level::warn, fmt_str, ##__VA_ARGS__);      \
    }

#define ML_ERROR_DEBUG(fmt_str, ...)                            \
    if (MermanLogger::GetInstance().IsDebugMode())       \
    {                                                          \
        MERMAN_LOGGER_CALL_(                                   \
            MermanLogger::GetInstance()->GetLogger(       \
                MERMAN_MODULE_NAME),                           \
            spdlog::level::err, fmt_str, ##__VA_ARGS__);       \
    }

// ============================================================
// Stream 流式打印宏
// ============================================================
#define ML_INFO_STREAM                                         \
    MermanLogStreamHelper(                                     \
        MermanLogger::GetInstance()->GetLogger(           \
            MERMAN_MODULE_NAME),                               \
        spdlog::level::info,                                   \
        spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__})

#define ML_WARNING_STREAM                                      \
    MermanLogStreamHelper(                                     \
        MermanLogger::GetInstance()->GetLogger(           \
            MERMAN_MODULE_NAME),                               \
        spdlog::level::warn,                                   \
        spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__})

#define ML_ERROR_STREAM                                        \
    MermanLogStreamHelper(                                     \
        MermanLogger::GetInstance()->GetLogger(           \
            MERMAN_MODULE_NAME),                               \
        spdlog::level::err,                                    \
        spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__})
