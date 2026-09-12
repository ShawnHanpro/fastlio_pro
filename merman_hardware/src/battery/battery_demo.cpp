#include "battery/battery_driver.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace {
volatile std::sig_atomic_t g_running = 1;

void handleSignal(int) {
    g_running = 0;
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try {
        BatteryConfig cfg;
        if (argc >= 2) {
            cfg = loadBatteryConfigFromYaml(argv[1]);
        } else {
            // Same defaults as battery.yaml. Change here if you do not pass yaml path.
            cfg.port = "/dev/ttyTHS1";
            cfg.baudrate = 9600;
            cfg.slave_id = 1;
            cfg.led_board_id = 2;
            cfg.poll_rate_hz = 1.0;
        }

        BatteryDriver driver(cfg);

        // Callback runs in battery worker thread. Keep it short.
        driver.setDataCallback([&driver](const BatteryData& data) {
            std::cout << BatteryDriver::toString(data)
                      << ", led_level=" << driver.socToLevel(data.soc)
                      << std::endl;
        });

        driver.setErrorCallback([](const std::string& err) {
            std::cerr << "Battery read/update failed: " << err << std::endl;
        });

        // start() will open serial port, then create internal loop thread:
        // read battery -> update SOC LED -> save latest data -> call callback.
        driver.start();

        std::cout << "Battery thread started. port=" << cfg.port
                  << ", baudrate=" << cfg.baudrate
                  << ", poll_rate_hz=" << cfg.poll_rate_hz
                  << std::endl;

        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // Example: other modules can fetch latest data at any time.
            // if (driver.hasLatestData()) {
            //     BatteryData latest = driver.latestData();
            // }
        }

        driver.stop();
        driver.close();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << std::endl;
        return 1;
    }
}
