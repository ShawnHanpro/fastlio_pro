#include "battery/battery_driver.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <utility>
#include <unistd.h>
#include <sys/ioctl.h>   // 声明 ioctl 函数
#include <termios.h>     // 定义 TIOCMGET, TIOCM_DTR 等串口控制宏

namespace {

std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::string stripComment(const std::string& line) {
    bool in_single_quote = false;
    bool in_double_quote = false;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '\'' && !in_double_quote) {
            in_single_quote = !in_single_quote;
        } else if (c == '"' && !in_single_quote) {
            in_double_quote = !in_double_quote;
        } else if (c == '#' && !in_single_quote && !in_double_quote) {
            return line.substr(0, i);
        }
    }
    return line;
}

std::string unquote(std::string value) {
    value = trim(value);
    if (value.size() >= 2) {
        const char first = value.front();
        const char last = value.back();
        if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
            return value.substr(1, value.size() - 2);
        }
    }
    return value;
}

bool parseBool(const std::string& value) {
    std::string v = trim(unquote(value));
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return v == "true" || v == "1" || v == "yes" || v == "on";
}

speed_t baudToConstant(int baudrate) {
    switch (baudrate) {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default:
            throw BatteryException("Unsupported baudrate: " + std::to_string(baudrate));
    }
}

uint16_t levelToMask(int level) {
    switch (level) {
        case 0: return 0x0000;
        case 1: return 0x0001;
        case 2: return 0x0003;
        case 3: return 0x0007;
        case 4: return 0x000F;
        case 5: return 0x001F;
        default:
            throw BatteryException("Invalid LED level, expected 0~5, got " + std::to_string(level));
    }
}

uint16_t rgbToValue(RgbState state) {
    switch (state) {
        case RgbState::Off: return 0x0000;
        case RgbState::Red: return 0x0001;
        case RgbState::Green: return 0x0002;
        case RgbState::Blue: return 0x0004;
    }
    return 0x0000;
}

}  // namespace

BatteryDriver::BatteryDriver(BatteryConfig config) : config_(std::move(config)) {}

BatteryDriver::~BatteryDriver() {
    stop();
    close();
}

void BatteryDriver::open()
{
    std::lock_guard<std::mutex> lk(io_mutex_);

    if (fd_ >= 0) {
        return;
    }

    fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        throw BatteryException(
            "Failed to open serial port " + config_.port + ": " + std::strerror(errno)
        );
    }

    configureSerialPort();

    // 关键：模拟 pyserial 打开串口后的 DTR/RTS 状态
    setModemLinesLikePyserial();

    ::tcflush(fd_, TCIOFLUSH);

    std::cout << "BatteryDriver::open success, fd_=" << fd_
              << ", port=" << config_.port << std::endl;
}

void BatteryDriver::close() {
    // Never close the fd while the worker thread may be using it.
    if (running_.load()) {
        stop();
    }

    std::lock_guard<std::mutex> lk(io_mutex_);
    if (fd_ >= 0) {
        ::tcdrain(fd_);
        ::close(fd_);
        fd_ = -1;
    }
}

bool BatteryDriver::isOpen() const {
    std::lock_guard<std::mutex> lk(io_mutex_);
    return fd_ >= 0;
}

void BatteryDriver::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }

    try {
        if (!isOpen()) {
            open();
        }
        worker_thread_ = std::thread(&BatteryDriver::workerLoop, this);
    } catch (...) {
        running_.store(false);
        stop_cv_.notify_all();
        throw;
    }
}

void BatteryDriver::stop() {
    const bool was_running = running_.exchange(false);
    stop_cv_.notify_all();

    if (worker_thread_.joinable()) {
        if (worker_thread_.get_id() == std::this_thread::get_id()) {
            // Do not join self. This should normally not happen unless stop() is called in a callback.
            worker_thread_.detach();
        } else {
            worker_thread_.join();
        }
    }

    if (was_running && config_.led_off_on_stop && isOpen()) {
        try {
            turnLedOff();
        } catch (...) {
            // Destructors/stop paths should not throw due to optional LED cleanup.
        }
    }
}

bool BatteryDriver::isRunning() const {
    return running_.load();
}

bool BatteryDriver::hasLatestData() const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return has_latest_data_;
}

BatteryData BatteryDriver::latestData() const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (!has_latest_data_) {
        throw BatteryException("No battery data has been received yet");
    }
    return latest_data_;
}

std::string BatteryDriver::lastError() const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return last_error_;
}

void BatteryDriver::setDataCallback(DataCallback callback) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    data_callback_ = std::move(callback);
}

void BatteryDriver::setErrorCallback(ErrorCallback callback) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    error_callback_ = std::move(callback);
}

std::chrono::milliseconds BatteryDriver::pollInterval() const {
    const double hz = config_.poll_rate_hz;
    if (hz <= 0.0) {
        return std::chrono::milliseconds(1000);
    }
    const auto ms = static_cast<int>(1000.0 / hz);
    return std::chrono::milliseconds(std::max(10, ms));
}

void BatteryDriver::workerLoop() {
    while (running_.load()) {
        const auto loop_start = std::chrono::steady_clock::now();

        try {
            BatteryData data = readAndUpdateLedOnce();

            DataCallback cb;
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                latest_data_ = data;
                has_latest_data_ = true;
                last_error_.clear();
                cb = data_callback_;
            }

            if (cb) {
                cb(data);
            }
        } catch (const std::exception& e) {
            ErrorCallback cb;
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                last_error_ = e.what();
                cb = error_callback_;
            }

            if (cb) {
                cb(e.what());
            }
        }

        const auto elapsed = std::chrono::steady_clock::now() - loop_start;
        const auto interval = pollInterval();
        const auto wait_time = elapsed >= interval
            ? std::chrono::milliseconds(1)
            : std::chrono::duration_cast<std::chrono::milliseconds>(interval - elapsed);

        std::mutex dummy_mutex;
        std::unique_lock<std::mutex> lk(dummy_mutex);
        stop_cv_.wait_for(lk, wait_time, [this]() { return !running_.load(); });
    }
}

void BatteryDriver::setConfig(const BatteryConfig& config) {
    if (running_.load()) {
        throw BatteryException("setConfig() is not allowed while battery thread is running");
    }

    std::lock_guard<std::mutex> lk(io_mutex_);
    if (fd_ >= 0) {
        throw BatteryException("setConfig() is not allowed while serial port is open");
    }
    config_ = config;
    last_led_level_ = -1;
    last_led_send_time_ = {};
    last_rgb_state_ = RgbState::Off;
    has_last_rgb_state_ = false;
}

void BatteryDriver::configureSerialPort()
{
    ensureOpen();

    termios tio{};
    if (::tcgetattr(fd_, &tio) != 0) {
        throw BatteryException("tcgetattr failed: " + std::string(std::strerror(errno)));
    }

    ::cfmakeraw(&tio);

    const speed_t speed = baudToConstant(config_.baudrate);
    if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0) {
        throw BatteryException("cfset speed failed: " + std::string(std::strerror(errno)));
    }

    tio.c_cflag |= CLOCAL;
    tio.c_cflag |= CREAD;

    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    tio.c_cflag &= ~PARENB;  // no parity
    tio.c_cflag &= ~CSTOPB;  // 1 stop bit

#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS; // no hardware flow control
#endif

    tio.c_iflag &= ~(IXON | IXOFF | IXANY);

    // pyserial timeout 类似行为
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = static_cast<cc_t>(std::max(1.0, config_.serial_timeout_sec * 10.0));

    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
        throw BatteryException("tcsetattr failed: " + std::string(std::strerror(errno)));
    }

    ::tcflush(fd_, TCIOFLUSH);
}

void BatteryDriver::ensureOpen() const {
    if (fd_ < 0) {
        throw BatteryException("Battery serial port is not open");
    }
}

uint16_t BatteryDriver::crc16Modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint16_t BatteryDriver::crc16Modbus(const std::vector<uint8_t>& data) {
    return crc16Modbus(data.data(), data.size());
}

void BatteryDriver::appendCrc(std::vector<uint8_t>& frame) {
    const uint16_t crc = crc16Modbus(frame);
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
}

std::vector<uint8_t> BatteryDriver::makeReadCommand() const {
    std::vector<uint8_t> frame = {
        static_cast<uint8_t>(config_.slave_id),
        0x04,
        static_cast<uint8_t>((config_.start_register >> 8) & 0xFF),
        static_cast<uint8_t>(config_.start_register & 0xFF),
        static_cast<uint8_t>((config_.register_count >> 8) & 0xFF),
        static_cast<uint8_t>(config_.register_count & 0xFF),
    };
    appendCrc(frame);
    return frame;
}

std::vector<uint8_t> BatteryDriver::makeWriteCommand(uint8_t board_id,
                                                     uint16_t address,
                                                     uint16_t value) const {
    std::vector<uint8_t> frame = {
        board_id,
        0x05,
        static_cast<uint8_t>((address >> 8) & 0xFF),
        static_cast<uint8_t>(address & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF),
    };
    appendCrc(frame);
    return frame;
}

std::vector<uint8_t> BatteryDriver::makeLedLevelCommand(int level) const
{
    switch (level) {
        case 0:
            return {0x02, 0x05, 0x00, 0x03, 0x00, 0x00, 0x3D, 0xF9}; // BAT_LEDOFF
        case 1:
            return {0x02, 0x05, 0x00, 0x03, 0x00, 0x01, 0xFC, 0x39}; // BAT LEVEL1
        case 2:
            return {0x02, 0x05, 0x00, 0x03, 0x00, 0x03, 0x7D, 0xF8}; // BAT LEVEL2
        case 3:
            return {0x02, 0x05, 0x00, 0x03, 0x00, 0x07, 0x7C, 0x3B}; // BAT LEVEL3
        case 4:
            return {0x02, 0x05, 0x00, 0x03, 0x00, 0x0F, 0x7D, 0xFD}; // BAT LEVEL4
        case 5:
            return {0x02, 0x05, 0x00, 0x03, 0x00, 0x1F, 0x7C, 0x31}; // BAT LEVEL5
        default:
            throw BatteryException("Invalid LED level, expected 0~5, got " + std::to_string(level));
    }
}

std::vector<uint8_t> BatteryDriver::makeRgbCommand(RgbState state) const
{
    switch (state) {
        case RgbState::Red:
            return {0x02, 0x05, 0x00, 0x02, 0x00, 0x01, 0xAD, 0xF9};
        case RgbState::Green:
            return {0x02, 0x05, 0x00, 0x02, 0x00, 0x02, 0xED, 0xF8};
        case RgbState::Blue:
            return {0x02, 0x05, 0x00, 0x02, 0x00, 0x04, 0x6D, 0xFA};
        case RgbState::Off:
        default:
            return {0x02, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x39};
    }
}

void BatteryDriver::writeAllLocked(const std::vector<uint8_t>& data) {
    ensureOpen();
    size_t written_total = 0;
    const auto start = std::chrono::steady_clock::now();
    const double timeout_sec = std::max(0.2, config_.serial_timeout_sec);

    while (written_total < data.size()) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        const double remaining = timeout_sec - elapsed;
        if (remaining <= 0.0) {
            throw BatteryException("Serial write timeout, tx=" + bytesToHex(data));
        }

        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(fd_, &wfds);
        timeval tv{};
        tv.tv_sec = static_cast<long>(remaining);
        tv.tv_usec = static_cast<long>((remaining - tv.tv_sec) * 1000000.0);

        const int ret = ::select(fd_ + 1, nullptr, &wfds, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw BatteryException("select(write) failed: " + std::string(std::strerror(errno)));
        }
        if (ret == 0) {
            continue;
        }

        const ssize_t n = ::write(fd_, data.data() + written_total, data.size() - written_total);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            throw BatteryException("serial write failed: " + std::string(std::strerror(errno)));
        }
        written_total += static_cast<size_t>(n);
    }

    if (::tcdrain(fd_) != 0) {
        throw BatteryException("tcdrain failed: " + std::string(std::strerror(errno)));
    }
}

std::vector<uint8_t> BatteryDriver::readBytesLocked(size_t expected_len, double timeout_sec)
{
    ensureOpen();

    std::vector<uint8_t> result;
    result.reserve(expected_len);

    const auto start = std::chrono::steady_clock::now();

    while (result.size() < expected_len) {
        uint8_t ch = 0;
        ssize_t n = ::read(fd_, &ch, 1);

        if (n == 1) {
            result.push_back(ch);
            continue;
        }

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw BatteryException("serial read failed: " + std::string(std::strerror(errno)));
        }

        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed >= timeout_sec) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return result;
}

std::vector<uint8_t> BatteryDriver::sendRawAndReadEchoLocked(
    const std::vector<uint8_t>& cmd,
    size_t echo_len,
    double echo_timeout_sec)
{
    (void)echo_timeout_sec;

    ensureOpen();

    ssize_t n = ::write(fd_, cmd.data(), cmd.size());
    if (n < 0) {
        throw BatteryException("serial write failed: " + std::string(std::strerror(errno)));
    }

    if (static_cast<size_t>(n) != cmd.size()) {
        throw BatteryException("serial write incomplete, tx=" + bytesToHex(cmd));
    }

    // 对齐 Python: self.ser.flush()
    if (::tcdrain(fd_) != 0) {
        throw BatteryException("tcdrain failed: " + std::string(std::strerror(errno)));
    }

    // 对齐 Python: time.sleep(0.03)
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // 对齐 Python: self.ser.read(8)
    return readBytesLocked(echo_len, config_.serial_timeout_sec);
}

BatteryData BatteryDriver::readBatteryOnce() {
    std::lock_guard<std::mutex> lk(io_mutex_);
    ensureOpen();

    const std::vector<uint8_t> cmd = makeReadCommand();
    const size_t expected_len = 3 + static_cast<size_t>(config_.register_count) * 2 + 2;

    ::tcflush(fd_, TCIFLUSH);
    writeAllLocked(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const std::vector<uint8_t> resp = readBytesLocked(expected_len, config_.serial_timeout_sec);

    if (resp.size() >= 5 && resp[0] == static_cast<uint8_t>(config_.slave_id) && resp[1] == 0x84) {
        std::ostringstream oss;
        oss << "Battery returned exception code: 0x" << std::hex << std::uppercase
            << std::setw(2) << std::setfill('0') << static_cast<int>(resp[2])
            << ", rx=" << bytesToHex(resp);
        throw BatteryException(oss.str());
    }

    if (resp.size() != expected_len) {
        throw BatteryException("Invalid response length: expected=" + std::to_string(expected_len) +
                               ", got=" + std::to_string(resp.size()) +
                               ", rx=" + bytesToHex(resp));
    }
    if (resp[0] != static_cast<uint8_t>(config_.slave_id)) {
        throw BatteryException("Invalid slave id: expected=" + std::to_string(config_.slave_id) +
                               ", got=" + std::to_string(resp[0]) +
                               ", rx=" + bytesToHex(resp));
    }
    if (resp[1] != 0x04) {
        std::ostringstream oss;
        oss << "Invalid function code: expected=0x04, got=0x" << std::hex << std::uppercase
            << std::setw(2) << std::setfill('0') << static_cast<int>(resp[1])
            << ", rx=" << bytesToHex(resp);
        throw BatteryException(oss.str());
    }
    if (resp[2] != static_cast<uint8_t>(config_.register_count * 2)) {
        throw BatteryException("Invalid byte count: expected=" + std::to_string(config_.register_count * 2) +
                               ", got=" + std::to_string(resp[2]) +
                               ", rx=" + bytesToHex(resp));
    }

    const uint16_t recv_crc = static_cast<uint16_t>(resp[resp.size() - 2]) |
                              (static_cast<uint16_t>(resp[resp.size() - 1]) << 8);
    const uint16_t calc_crc = crc16Modbus(resp.data(), resp.size() - 2);
    if (recv_crc != calc_crc) {
        std::ostringstream oss;
        oss << "CRC error: recv=0x" << std::hex << std::uppercase << std::setw(4)
            << std::setfill('0') << recv_crc << ", calc=0x" << std::setw(4) << calc_crc
            << ", rx=" << bytesToHex(resp);
        throw BatteryException(oss.str());
    }

    std::vector<uint16_t> regs;
    regs.reserve(config_.register_count);
    const size_t data_begin = 3;
    const size_t data_end = resp.size() - 2;
    for (size_t i = data_begin; i + 1 < data_end; i += 2) {
        regs.push_back(static_cast<uint16_t>((resp[i] << 8) | resp[i + 1]));
    }
    if (regs.size() < 13) {
        throw BatteryException("Not enough registers decoded: " + std::to_string(regs.size()));
    }

    BatteryData data;
    data.voltage = regs[0] * 0.01;
    data.current = (static_cast<int>(regs[1]) - 10000) * 0.1;
    data.remain_ah = regs[2] * 0.1;
    data.avg_temp = (static_cast<int>(regs[3]) - 400) * 0.1;
    data.env_temp = (static_cast<int>(regs[4]) - 400) * 0.1;
    data.alarm = regs[5];
    data.protect = regs[6];
    data.fault_status = regs[7];
    data.soc = regs[8] * 0.01;
    data.soh = regs[9] * 0.01;
    data.cycles = regs[10];
    data.backup_time = regs[11] * 0.01;
    data.work_mode = regs[12];
    return data;
}

BatteryData BatteryDriver::readAndUpdateLedOnce() {
    BatteryData data = readBatteryOnce();
    updateLedBySoc(data.soc);
    return data;
}

void BatteryDriver::setLedLevel(int level) {
    std::lock_guard<std::mutex> lk(io_mutex_);
    ensureOpen();

    const std::vector<uint8_t> cmd = makeLedLevelCommand(level);

    std::cout << "LED LEVEL TX level=" << level
              << ", board_id=" << config_.led_board_id
              << ", cmd=" << bytesToHex(cmd)
              << std::endl;

    // LED 板可能回显 8 字节，也可能不回显。
    std::vector<uint8_t> echo = sendRawAndReadEchoLocked(cmd, 8, 0.08);

    if (echo.empty()) {
        std::cout << "LED LEVEL RX: <empty>" << std::endl;
    } else {
        std::cout << "LED LEVEL RX: " << bytesToHex(echo) << std::endl;
    }

    last_led_level_ = level;
    last_led_send_time_ = std::chrono::steady_clock::now();
}

void BatteryDriver::turnLedOff() {
    setLedLevel(0);
}

void BatteryDriver::turnLedFullOn() {
    setLedLevel(5);
}

void BatteryDriver::setRgbState(RgbState state) {
    std::lock_guard<std::mutex> lk(io_mutex_);
    ensureOpen();
    const std::vector<uint8_t> cmd = makeRgbCommand(state);
    (void)sendRawAndReadEchoLocked(cmd, 8, 0.08);
    last_rgb_state_ = state;
    has_last_rgb_state_ = true;
}

bool BatteryDriver::updateLedBySoc(double soc, bool force) {
    if (!config_.led_enabled) {
        return false;
    }

    bool sent = false;
    const int level = socToLevel(soc);
    const auto now = std::chrono::steady_clock::now();
    const bool never_sent = last_led_level_ < 0 || last_led_send_time_ == std::chrono::steady_clock::time_point{};
    const double elapsed = never_sent ? 1e9 : std::chrono::duration<double>(now - last_led_send_time_).count();
    const bool heartbeat_due = elapsed >= config_.led_heartbeat_sec;
    const bool changed = level != last_led_level_;

    if (force || !config_.led_update_on_change_only || changed || heartbeat_due) {
        setLedLevel(level);
        sent = true;
    }

    if (config_.rgb_status_enabled) {
        const RgbState rgb_state = (soc <= config_.rgb_low_soc_threshold) ? RgbState::Red : RgbState::Green;
        if (force || !has_last_rgb_state_ || rgb_state != last_rgb_state_ || heartbeat_due) {
            setRgbState(rgb_state);
            sent = true;
        }
    }

    return sent;
}

int BatteryDriver::socToLevel(double soc) const {
    if (soc <= 0.0) {
        return 0;
    }
    if (soc <= config_.level1_max_soc) {
        return 1;
    }
    if (soc <= config_.level2_max_soc) {
        return 2;
    }
    if (soc <= config_.level3_max_soc) {
        return 3;
    }
    if (soc <= config_.level4_max_soc) {
        return 4;
    }
    return 5;
}

std::string BatteryDriver::decodeWorkMode(int mode) {
    switch (mode) {
        case 0x0001: return "standby";
        case 0x0002: return "charging";
        case 0x0003: return "fully_charged";
        case 0x0004: return "discharge_complete";
        default: return "unknown";
    }
}

std::string BatteryDriver::toString(const BatteryData& data) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "V=" << data.voltage << "V, "
        << "I=" << data.current << "A, "
        << "SOC=" << data.soc << "%, "
        << "SOH=" << data.soh << "%, "
        << "Remain=" << data.remain_ah << "Ah, "
        << std::setprecision(1)
        << "Tavg=" << data.avg_temp << "C, "
        << "Tenv=" << data.env_temp << "C, "
        << "cycles=" << data.cycles << ", "
        << "mode=0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
        << data.work_mode << "(" << decodeWorkMode(data.work_mode) << "), "
        << "alarm=0x" << std::setw(4) << data.alarm << ", "
        << "protect=0x" << std::setw(4) << data.protect << ", "
        << "fault_status=0x" << std::setw(4) << data.fault_status;
    return oss.str();
}

std::string BatteryDriver::bytesToHex(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    oss << std::hex << std::uppercase << std::setfill('0');
    for (size_t i = 0; i < data.size(); ++i) {
        if (i > 0) {
            oss << ' ';
        }
        oss << std::setw(2) << static_cast<int>(data[i]);
    }
    return oss.str();
}

BatteryConfig loadBatteryConfigFromYaml(const std::string& yaml_path) {
    std::ifstream fin(yaml_path);
    if (!fin.is_open()) {
        throw BatteryException("Failed to open yaml config: " + yaml_path);
    }

    BatteryConfig cfg;
    std::string line;
    while (std::getline(fin, line)) {
        line = trim(stripComment(line));
        if (line.empty()) {
            continue;
        }

        const auto pos = line.find(':');
        if (pos == std::string::npos) {
            continue;
        }

        const std::string key = trim(line.substr(0, pos));
        std::string value = trim(line.substr(pos + 1));
        if (value.empty()) {
            continue;
        }
        value = unquote(value);

        try {
            if (key == "port") cfg.port = value;
            else if (key == "baudrate") cfg.baudrate = std::stoi(value);
            else if (key == "slave_id") cfg.slave_id = std::stoi(value);
            else if (key == "serial_timeout") cfg.serial_timeout_sec = std::stod(value);
            else if (key == "publish_rate") cfg.poll_rate_hz = std::stod(value);
            else if (key == "poll_rate_hz") cfg.poll_rate_hz = std::stod(value);
            else if (key == "start_register") cfg.start_register = static_cast<uint16_t>(std::stoi(value));
            else if (key == "register_count") cfg.register_count = static_cast<uint16_t>(std::stoi(value));
            else if (key == "led_enabled") cfg.led_enabled = parseBool(value);
            else if (key == "led_board_id") cfg.led_board_id = std::stoi(value);
            else if (key == "led_update_on_change_only") cfg.led_update_on_change_only = parseBool(value);
            else if (key == "led_heartbeat_sec") cfg.led_heartbeat_sec = std::stod(value);
            else if (key == "level1_max_soc") cfg.level1_max_soc = std::stod(value);
            else if (key == "level2_max_soc") cfg.level2_max_soc = std::stod(value);
            else if (key == "level3_max_soc") cfg.level3_max_soc = std::stod(value);
            else if (key == "level4_max_soc") cfg.level4_max_soc = std::stod(value);
            else if (key == "rgb_status_enabled") cfg.rgb_status_enabled = parseBool(value);
            else if (key == "rgb_low_soc_threshold") cfg.rgb_low_soc_threshold = std::stod(value);
            else if (key == "led_off_on_stop") cfg.led_off_on_stop = parseBool(value);
        } catch (const std::exception& e) {
            throw BatteryException("Invalid value in yaml for key '" + key + "': " + value + ", " + e.what());
        }
    }

    return cfg;
}

void BatteryDriver::setModemLinesLikePyserial()
{
    int status = 0;
    if (::ioctl(fd_, TIOCMGET, &status) == -1) {
        std::cerr << "TIOCMGET failed: " << std::strerror(errno) << std::endl;
        return;
    }

    // pyserial 通常打开后 DTR/RTS 是 active 状态
    status |= TIOCM_DTR;
    status |= TIOCM_RTS;

    if (::ioctl(fd_, TIOCMSET, &status) == -1) {
        std::cerr << "TIOCMSET failed: " << std::strerror(errno) << std::endl;
        return;
    }

    std::cout << "Serial modem lines set: DTR=1, RTS=1" << std::endl;
}



void BatteryDriver::testPythonLedFullOn()
{
    std::lock_guard<std::mutex> lk(io_mutex_);
    ensureOpen();

    const std::vector<uint8_t> cmd = {
        0x02, 0x05, 0x00, 0x03, 0x00, 0x1F, 0x7C, 0x31
    };

    std::cout << "PYTHON STYLE LED TX: " << bytesToHex(cmd) << std::endl;

    auto rx = sendRawAndReadEchoLocked(cmd, 8, config_.serial_timeout_sec);

    if (rx.empty()) {
        std::cout << "PYTHON STYLE LED RX: <empty>" << std::endl;
    } else {
        std::cout << "PYTHON STYLE LED RX: " << bytesToHex(rx) << std::endl;
    }
}