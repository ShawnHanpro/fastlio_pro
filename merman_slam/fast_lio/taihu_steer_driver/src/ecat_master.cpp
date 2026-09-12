#include "ecat_master.hpp"

EcatMaster::EcatMaster(int master_id):task(master_id)
{
}

void EcatMaster::init(int affinity, int priority, int interval, std::int64_t cycle_time, 
    std::int64_t shiftTime, const std::string& fileName)
{
  task.priority(priority);
  cpu_set_t cpus;
  CPU_ZERO(&cpus);
  CPU_SET(affinity, &cpus);
  task.cpu_affinity(&cpus, sizeof(cpus));
  task.set_interval(interval);
  run_period = cycle_time;

  if (!fileName.empty())
  {
    //eni模式下直接获取指定的xml文件中的配置
    std::cout << "Use eni xml\n";
    task.load_eni(fileName, cycle_time);
    run_period = cycle_time;
  }
  else
  {
    //esi模式下设置默认的循环周期和同步模式
    std::cout << "Use esi xml\n";
    task.cycle_time(cycle_time, shiftTime);
    task.dc_mode(ecat::dc_mode::master_follow_slave);
  }

  //设置config的回调函数，将在task.start的时候被到用
  task.set_config_callback([&] {
    std::uint16_t slave_count = task.slave_count();
    std::uint16_t slave_pos = 0;
    for (slave_pos = 0; slave_pos < slave_count; slave_pos++)
    {
      //获取slave对应的运行协议，如果还未设置则默认为0
      auto profile_no = task.profile_no(slave_pos);
      //CiA402协议模式
      if (profile_no == 402)
      {
        int n_axis_in_slave = 1, slot_pos;
        int slots_count = task.slots_count(slave_pos);
        int slots_index_increment = task.slot_index_increment(slave_pos);
        if (slots_count > 0)
        {
          assert(slots_index_increment != -1);
          n_axis_in_slave = slots_count;
        }
        assert(!(slots_count < 0));
        for (slot_pos = 0; slot_pos < n_axis_in_slave; ++slot_pos)
        {
          const int index_offset = slot_pos * slots_index_increment;
          auto axis = std::make_unique<axis_data>();
          axis->slave_pos = slave_pos;
          axis->axis_id = axis_count++;

          task.try_register_pdo_entry(axis->control_word, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6040 + index_offset), 0}); // control word;
          task.try_register_pdo_entry(axis->target_position, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x607a + index_offset), 0}); // target position
          task.try_register_pdo_entry(axis->target_velocity, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x60ff + index_offset), 0}); // target velocity
          task.try_register_pdo_entry(axis->target_torque, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6071 + index_offset), 0}); // target torque
          task.try_register_pdo_entry(axis->mode_of_operation, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6060 + index_offset), 0}); // mode of operation
          
          task.try_register_pdo_entry(axis->status_word, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6041 + index_offset), 0}); // status word
          task.try_register_pdo_entry(axis->position_actual, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6064 + index_offset), 0}); // position actual value
          task.try_register_pdo_entry(axis->velocity_actual, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x606c + index_offset), 0}); // velocity actual value
          task.try_register_pdo_entry(axis->torque_actual, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6077 + index_offset), 0}); // torque actual value
          task.try_register_pdo_entry(axis->mode_of_operation_display, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6061 + index_offset), 0}); // mode of operation display
          task.try_register_pdo_entry(axis->error_code, slave_pos,
                                        {static_cast<ecat::pdo_index_type>(0x603f + index_offset), 0});
          axes.push_back(std::move(axis));
          enable_count.push_back(0);
          error.push_back(0);
          motor_offset.push_back(0);
          motor_dir.push_back(0);
        }
      }
    }
    ecat::S2SConfig::use().count();
  });
}

void EcatMaster::start()
{
  task.start();
}

void EcatMaster::startAsMaster()  
{
  task.setAsMaster();
}

bool EcatMaster::startAsSlave(EcatMaster* master)
{
  printf("..................  startAsSlave, period: %d, master period: %d...\n", run_period, master->run_period);
  if(run_period < master->run_period)
  {
    printf("Start ec as slave failed, the period(%d) must >= the master period(%d).\n", run_period, master->run_period);
    return false;
  }

  if(run_period % master->run_period != 0)
  {
    printf("Start ec as slave failed,  period(%d) must be times of master period(%d).\n", run_period, master->run_period);
    return false;
  }

  task.setAsSlave(&master->task, run_period / master->run_period);
  return true;
}

bool EcatMaster::startAsSlave(EcatMaster* master, int num)
{
  printf("..................  startAsSlave, period: %d, master period: %d...\n", run_period, master->run_period);
  if(run_period < master->run_period)
  {
    printf("Start ec as slave failed, the period(%d) must >= the master period(%d).\n", run_period, master->run_period);
    return false;
  }

  if(run_period % master->run_period != 0)
  {
    printf("Start ec as slave failed,  period(%d) must be times of master period(%d).\n", run_period, master->run_period);
    return false;
  }

  task.setAsSlave(&master->task, run_period / master->run_period, num);
  return true;
}

void EcatMaster::wait()
{
    task.wait();
}

void EcatMaster::release()
{
    task.release();
}

void EcatMaster::disable_slaves()
{
    for(int i_slave=0; i_slave<axis_count; i_slave++)
        disable_slave(i_slave);
}

void EcatMaster::reset_faults()
{
    for(int i_slave=0; i_slave<axis_count; i_slave++)
      reset_fault(i_slave);
}

void EcatMaster::reset_fault(int slaveid)
{
    *axes[slaveid]->control_word = 128;
    enable_count[slaveid] = 0;
}


void EcatMaster::disable_slave(int slaveid)
{
    *axes[slaveid]->control_word = 0;
    enable_count[slaveid] = 0;
}

void EcatMaster::set_control_modes(int mode)
{
  for(int i_slave=0; i_slave<axis_count; i_slave++)
      *axes[i_slave]->mode_of_operation = static_cast<int8_t>(mode);
}

void EcatMaster::set_control_mode(int slaveid, int mode)
{
    *axes[slaveid]->mode_of_operation = static_cast<int8_t>(mode);
}

void EcatMaster::enable_slaves()
{
    for(int i_slave=0; i_slave<axis_count; i_slave++)
      enable_slave(i_slave);    
}

void EcatMaster::enable_slave(int slaveid)
{
  if(enable_count[slaveid]<100)
    *axes[slaveid]->control_word = 6;
  else if(enable_count[slaveid]>=100 && enable_count[slaveid]<200)
    *axes[slaveid]->control_word = 134;
  else if(enable_count[slaveid]>=200 && enable_count[slaveid]<300)
    *axes[slaveid]->control_word = 7;
  else if(enable_count[slaveid]>=300 && enable_count[slaveid]<400)
    *axes[slaveid]->control_word = 15;
  // else if(enable_count[slaveid]>=400)
  //   *axes[slaveid]->control_word = 31;

  if(enable_count[slaveid]<400)
      enable_count[slaveid]++;
}

bool EcatMaster::check_slaves_enable()
{
    bool all_enabled = true;
    for(int idx=0; idx<axis_count; idx++)
        all_enabled = all_enabled && check_slave_enable(idx);

    return all_enabled;
}

bool EcatMaster::check_slave_enable(int slaveid)
{
    if((*axes[slaveid]->status_word & 0xf)==7
        && ((*axes[slaveid]->status_word>>5) & 3)==1)
        return true;
    else
        return false;
}

void EcatMaster::set_motor_config(int *g_motor_offset, int *g_motor_dir, int num_motors)
{
  std::cout<<"offset: ";
  for(int i = 0; i<num_motors;i++)
  {
    motor_offset[i] = g_motor_offset[i];
    motor_dir[i] = g_motor_dir[i];
    std::cout<<g_motor_offset[i]<<", ";
  }
  std::cout<<std::endl;
}

EcatMaster::~EcatMaster()
{
    task.break_();
}