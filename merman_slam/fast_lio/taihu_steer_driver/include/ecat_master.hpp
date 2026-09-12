#include "ecat/task.hpp"
#include <qiuniu/init.h>
#include <vector>
#include "ecat/s2s_func.hpp"

#include <iostream>
#include <spdlog/spdlog.h>
#include <optional>
#include <ecat/types.hpp>
#include <map>
#include <Eigen/Dense>

struct axis_data
{
  std::uint16_t axis_id;
  std::uint16_t slave_pos;

  // process data address
  volatile std::uint16_t *control_word; //6040
  volatile std::int32_t *target_position; //607A
  volatile std::int32_t *target_velocity; //0x60ff
  volatile std::int16_t *target_torque; //6071  千分之一最大转矩
  volatile std::int8_t *mode_of_operation;//6060

  // volatile const std::uint16_t *error_code;
  volatile const std::uint16_t *status_word; //6041
  volatile const std::int32_t *position_actual;//6064
  volatile const std::int32_t *velocity_actual;//606c
  volatile const std::int16_t *torque_actual;//6077 千分之一最大转矩
  volatile const std::int8_t *mode_of_operation_display;//6061

  volatile const std::uint16_t *error_code;//603f

  std::uint16_t old_status_word;
};

enum class cia402_state
{
  none = -1,
  not_ready_to_switch_on = 0,
  switch_on_disabled,
  ready_to_switch_on,
  switched_on,
  operation_enabled,
  quick_stop_active,
  fault_reaction_active,
  fault,
};

enum mode_of_operation_type : std::int8_t
{
  op_mode_pt = -1,
  op_mode_no = 0,
  op_mode_pp = 1,
  op_mode_vl = 2,
  op_mode_pv = 3,
  op_mode_hm = 6,
  op_mode_ip = 7,
  op_mode_csp = 8,
  op_mode_csv = 9,
  op_mode_cst = 10,
};

class EcatMaster{
public:
    int run_period = 5000000;
    int axis_count = 0; //电机数
    std::vector<std::unique_ptr<axis_data>> axes; // 轴数据
    ecat::task task;

public:
    std::vector<int> error;
    std::vector<int> motor_offset;
    std::vector<int> motor_dir;
private:
    std::vector<int> enable_count;
public:
    EcatMaster(int master_id);
    ~EcatMaster();

    void init(int affinity, int priority, int interval, std::int64_t cycle_time, 
      std::int64_t shiftTime, const std::string& fileName);
    void start();
    void startAsMaster();
    bool startAsSlave(EcatMaster* master);
    bool startAsSlave(EcatMaster* master, int num);
    void wait();
    void release();

    void disable_slaves();
    void disable_slave(int slaveid);
    void set_control_modes(int mode);
    void set_control_mode(int slaveid, int mode);
    void enable_slaves();
    void enable_slave(int slaveid);
    bool check_slaves_enable();
    bool check_slave_enable(int slaveid);
    void reset_faults();
    void reset_fault(int slaveid);

    void set_motor_config(int *g_motor_offset, int *g_motor_dir, int num_motors);
};
