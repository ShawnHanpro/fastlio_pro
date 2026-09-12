#pragma once

class BootInit{
public:
    BootInit();
    void SystemInit();
    bool get_exit_thread();

private:
    bool global_exit_thread_ = false;
};