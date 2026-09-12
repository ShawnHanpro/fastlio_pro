#include "main/boot_init.h"

BootInit::BootInit(){
    
}

void BootInit::SystemInit(){
    global_exit_thread_ = false;
}

bool BootInit::get_exit_thread(){
    return global_exit_thread_;
}