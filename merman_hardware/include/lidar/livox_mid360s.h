#include "lidar/lidar.h"

class LivoxMid360s : public Lidar {
public:
    static LivoxMid360s *GetInstance() {
        static LivoxMid360s instance_;
        return &instance_;
    }

    void run() override;

private:
    LivoxMid360s();
    ~LivoxMid360s()=default;

    void SendData();
};