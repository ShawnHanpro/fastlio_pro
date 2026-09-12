#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "merman_types.h"

class BatteryException : public std::runtime_error {
public:
    explicit BatteryException(const std::string& msg) : std::runtime_error(msg) {}
};

enum class RgbState {
    Off,
    Red,
    Green,
    Blue,
};

class BatteryDriver {
public:
    using DataCallback = std::function<void(const BatteryData&)>;
    using ErrorCallback = std::function<void(const std::string&)>;

    explicit BatteryDriver(BatteryConfig config = BatteryConfig{});
    ~BatteryDriver();

    BatteryDriver(const BatteryDriver&) = delete;
    BatteryDriver& operator=(const BatteryDriver&) = delete;

    // Open/close serial port. open() throws BatteryException on failure.
    void open();
    void close();
    bool isOpen() const;

    // Start/stop internal polling thread:
    // thread loop = read battery once -> update LED by SOC -> save latest data -> callback.
    void start();
    void stop();
    bool isRunning() const;

    // Battery polling interface. Throws BatteryException on serial/protocol errors.
    BatteryData readBatteryOnce();

    // Convenience: read battery once, then update LED according to SOC.
    BatteryData readAndUpdateLedOnce();

    // Latest data produced by internal thread.
    bool hasLatestData() const;
    BatteryData latestData() const;
    std::string lastError() const;

    // Optional callbacks. They are called from the battery worker thread.
    // Keep callback body short; do not call BatteryDriver methods inside callback.
    void setDataCallback(DataCallback callback);
    void setErrorCallback(ErrorCallback callback);

    // Direct LED control. These methods send immediately and do NOT check led_enabled.
    void setLedLevel(int level);   // level: 0~5, 0=off, 5=full bar
    void turnLedOff();
    void turnLedFullOn();
    void setRgbState(RgbState state);

    // LED policy used by readAndUpdateLedOnce(). Respects config.led_enabled.
    // Returns true if at least one LED/RGB command was sent.
    bool updateLedBySoc(double soc, bool force = false);

    int socToLevel(double soc) const;

    const BatteryConfig& config() const { return config_; }
    void setConfig(const BatteryConfig& config);

    static uint16_t crc16Modbus(const uint8_t* data, size_t len);
    static uint16_t crc16Modbus(const std::vector<uint8_t>& data);
    static void appendCrc(std::vector<uint8_t>& frame);
    static std::string decodeWorkMode(int mode);
    static std::string toString(const BatteryData& data);
    static std::string bytesToHex(const std::vector<uint8_t>& data);

    void setModemLinesLikePyserial();

    void testPythonLedFullOn();

private:
    BatteryConfig config_;
    int fd_ = -1;
    mutable std::mutex io_mutex_;

    std::atomic_bool running_{false};
    std::thread worker_thread_;
    mutable std::mutex state_mutex_;
    std::condition_variable stop_cv_;
    BatteryData latest_data_{};
    bool has_latest_data_ = false;
    std::string last_error_;
    DataCallback data_callback_;
    ErrorCallback error_callback_;

    int last_led_level_ = -1;
    std::chrono::steady_clock::time_point last_led_send_time_{};
    RgbState last_rgb_state_ = RgbState::Off;
    bool has_last_rgb_state_ = false;

    void workerLoop();
    std::chrono::milliseconds pollInterval() const;

    void configureSerialPort();
    void ensureOpen() const;

    std::vector<uint8_t> makeReadCommand() const;
    std::vector<uint8_t> makeWriteCommand(uint8_t board_id, uint16_t address, uint16_t value) const;
    std::vector<uint8_t> makeLedLevelCommand(int level) const;
    std::vector<uint8_t> makeRgbCommand(RgbState state) const;

    void writeAllLocked(const std::vector<uint8_t>& data);
    std::vector<uint8_t> readBytesLocked(size_t expected_len, double timeout_sec);
    std::vector<uint8_t> sendRawAndReadEchoLocked(const std::vector<uint8_t>& cmd,
                                                  size_t echo_len,
                                                  double echo_timeout_sec);
};

// Minimal parser for the provided ROS-style battery.yaml. No yaml-cpp dependency required.
// Unknown keys are ignored.
BatteryConfig loadBatteryConfigFromYaml(const std::string& yaml_path);
