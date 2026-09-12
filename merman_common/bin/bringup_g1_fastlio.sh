#!/bin/bash

# ============================================================
# Merman G1 精简启动脚本 - 分模块日志版
#
# 终端只显示启动摘要。
# 每个模块 stdout/stderr 独立写入日志：
#
#   logs/latest/
#   ├── hardware.log
#   ├── zlac.log
#   ├── taihu.log
#   ├── motor_enable.log
#   ├── livox.log
#   ├── localization.log
#   └── g1_nav2.log
#
# 不做 topic / TF 阻塞等待。
# 单个模块退出不会让整个 bringup 自动退出。
# ============================================================

ROS_DISTRO="${ROS_DISTRO:-humble}"

# WARNING: plaintext root password stored in this script.
# Replace the placeholder below with the actual root/sudo password.
ROOT_PASSWORD="1"

# sudo helper: read password from ROOT_PASSWORD instead of terminal input.
sudo_with_password()
{
    printf '%s\n' "${ROOT_PASSWORD}" | sudo -S -p '' "$@"
}

ROS2_BRIDGE_WS="/home/niic/slam_nav/merman_ros2_bridge"
FASTLIO_WS="/home/niic/slam_nav/merman_slam/fast_lio"
MERMAN_COMMON="/home/niic/slam_nav/merman_common"
MERMAN_ROBOT="${MERMAN_COMMON}/bin/merman_robot"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# ============================================================
# Fast DDS 传输配置
#
# Taihu 以 root 运行，而其它 ROS2 节点以 niic 用户运行。
# Fast DDS 默认可能优先使用 SHM；跨 Linux 用户时会出现：
#   root 能收到 /steer/joint_states
#   niic 收不到
#
# 实机验证：
#   FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ros2 topic echo ...
# 可以正常跨用户通信。
#
# 因此整个系统统一强制使用 UDPv4，避免 Fast DDS SHM 权限问题。
# ============================================================
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

# 新版 rcutils 环境变量，避免旧变量的 deprecated/ignored 提示。
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"

# Taihu EtherCAT bus is expected to contain exactly four steering motor slaves.
TAIHU_EXPECTED_SLAVES="${TAIHU_EXPECTED_SLAVES:-4}"
TAIHU_SLAVE_WAIT_SEC="${TAIHU_SLAVE_WAIT_SEC:-15}"
TAIHU_ETHERCAT_READY=0

PIDS=()
PID_NAMES=()


# ============================================================
# 日志目录
# ============================================================

LOG_ROOT="${MERMAN_COMMON}/logs"
RUN_ID="$(date '+%Y%m%d_%H%M%S')"
LOG_DIR="${LOG_ROOT}/${RUN_ID}"

mkdir -p "${LOG_DIR}"
mkdir -p "${LOG_ROOT}"

# logs/latest 永远指向最近一次启动
ln -sfn "${LOG_DIR}" "${LOG_ROOT}/latest"


# ============================================================
# ROS 环境
# ============================================================

source_if_exists()
{
    [ -f "$1" ] && source "$1"
}

source_if_exists "/opt/ros/${ROS_DISTRO}/setup.bash"
# source_if_exists "${ROS2_BRIDGE_WS}/install/setup.bash"
source_if_exists "${FASTLIO_WS}/install/setup.bash"


# ============================================================
# 第三方动态库
# ============================================================

# MERMAN_LIBRARY="/home/niic/slam_nav/merman_common/third_party/arm64"

# if [ -d "${MERMAN_LIBRARY}" ]; then
#     export LD_LIBRARY_PATH="${MERMAN_LIBRARY}/g2o/lib:\
# ${MERMAN_LIBRARY}/opencv/lib:\
# ${MERMAN_LIBRARY}/pcl/lib:\
# ${MERMAN_LIBRARY}/boost/lib:\
# ${MERMAN_LIBRARY}/flann/lib:\
# ${MERMAN_LIBRARY}/sqlite/lib:\
# ${MERMAN_LIBRARY}/libpointmatcher/lib:\
# ${LD_LIBRARY_PATH}"
# fi


# ============================================================
# 启动辅助
# ============================================================

add_pid()
{
    PIDS+=("$1")
    PID_NAMES+=("$2")
}


start_user()
{
    local name="$1"
    local logfile="$2"
    shift 2

    echo "[START] ${name}"
    echo "        log: ${logfile}"

    # 每个 ros2 launch 的 stdout + stderr 全部进入自己的日志文件。
    "$@" >"${logfile}" 2>&1 &

    local pid=$!
    add_pid "${pid}" "${name}"

    echo "        pid: ${pid}"
}


wait_taihu_slaves()
{
    local expected="${1:-4}"
    local timeout_sec="${2:-15}"
    local i
    local output
    local rc
    local count

    if ! command -v ethercat >/dev/null 2>&1; then
        echo "[ERROR] ethercat CLI not found; cannot check Taihu slave count"
        return 1
    fi

    echo "============================================================"
    echo "Taihu EtherCAT slave check"
    echo "expected: ${expected}"
    echo "timeout : ${timeout_sec}s"
    echo "============================================================"

    for ((i=1; i<=timeout_sec; ++i)); do
        output="$(sudo_with_password ethercat slaves 2>&1)"
        rc=$?

        if [ ${rc} -eq 0 ]; then
            # IgH `ethercat slaves` prints one slave per line.
            # Typical line begins with: 0  0:0  PREOP ...
            count="$(
                printf '%s\n' "${output}" |
                awk '$1 ~ /^[0-9]+$/ && $2 ~ /^[0-9]+:[0-9]+$/ {n++} END {print n+0}'
            )"
        else
            count=0
        fi

        echo
        echo "[CHECK ${i}/${timeout_sec}] Taihu EtherCAT slaves=${count}/${expected}"
        echo "${output}"

        if [ ${rc} -eq 0 ] && [ "${count}" -eq "${expected}" ]; then
            echo "[OK] Taihu EtherCAT slave count is exactly ${expected}"
            return 0
        fi

        sleep 1
    done

    echo
    echo "[ERROR] Taihu EtherCAT slave count did not reach exactly ${expected}"
    return 1
}


start_root_taihu()
{
    local logfile="${LOG_DIR}/taihu.log"

    echo "[START][root] Taihu steer driver"
    echo "              log: ${logfile}"

    # 当前 niic shell 负责打开 logfile，因此即使 ros2 本体以 root
    # 运行，日志文件仍然保存在当前 bringup 的统一目录中。
    sudo_with_password env \
        ROS_DISTRO="${ROS_DISTRO}" \
        FASTLIO_WS="${FASTLIO_WS}" \
        ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
        ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
        FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
        RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-}" \
        CYCLONEDDS_URI="${CYCLONEDDS_URI:-}" \
        FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-}" \
        FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-}" \
        LD_LIBRARY_PATH="${LD_LIBRARY_PATH}" \
        RCUTILS_LOGGING_USE_STDOUT=1 \
        RCUTILS_LOGGING_BUFFERED_STREAM=0 \
        bash -c '
            source "/opt/ros/${ROS_DISTRO}/setup.bash"
            source "${FASTLIO_WS}/install/setup.bash"

            export ROS_DOMAIN_ID
            export ROS_LOCALHOST_ONLY
            export FASTDDS_BUILTIN_TRANSPORTS
            export LD_LIBRARY_PATH
            export RCUTILS_LOGGING_USE_STDOUT
            export RCUTILS_LOGGING_BUFFERED_STREAM

            [ -n "${RMW_IMPLEMENTATION}" ] &&
                export RMW_IMPLEMENTATION

            [ -n "${CYCLONEDDS_URI}" ] &&
                export CYCLONEDDS_URI

            [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE}" ] &&
                export FASTRTPS_DEFAULT_PROFILES_FILE

            [ -n "${FASTDDS_DEFAULT_PROFILES_FILE}" ] &&
                export FASTDDS_DEFAULT_PROFILES_FILE

            exec ros2 launch \
                taihu_steer_driver \
                taihu_steer_driver.launch.py
        ' >"${logfile}" 2>&1 &

    local pid=$!
    add_pid "${pid}" "Taihu steer driver"

    echo "              pid: ${pid}"
}


# ============================================================
# 停止
# ============================================================

cleanup()
{
    echo
    echo "[STOP] stopping G1 bringup..."

    for pid in "${PIDS[@]}"; do
        kill -INT "${pid}" 2>/dev/null || true
    done

    # root Taihu
    sudo_with_password pkill -INT -f "taihu_steer_driver_node" 2>/dev/null || true
    sudo_with_password pkill -INT -f \
        "ros2 launch taihu_steer_driver taihu_steer_driver.launch.py" \
        2>/dev/null || true

    # EtherCAT
    if command -v ethercatctl >/dev/null 2>&1; then
        sudo_with_password ethercatctl stop >>"${LOG_DIR}/hardware.log" 2>&1 || true
    elif command -v ethercat >/dev/null 2>&1; then
        sudo_with_password ethercat stop >>"${LOG_DIR}/hardware.log" 2>&1 || true
    fi

    echo "[STOP] done"
    echo "[LOG] ${LOG_DIR}"
    exit 0
}

trap cleanup INT TERM


# ============================================================
# 启动摘要
# ============================================================

echo "============================================================"
echo " Merman G1 Bringup"
echo "============================================================"
echo "ROS_DOMAIN_ID             = ${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY        = ${ROS_LOCALHOST_ONLY}"
echo "RMW_IMPLEMENTATION        = ${RMW_IMPLEMENTATION:-default}"
echo "FASTDDS_BUILTIN_TRANSPORTS= ${FASTDDS_BUILTIN_TRANSPORTS}"
echo "LOG_DIR                   = ${LOG_DIR}"
echo "============================================================"

sudo_with_password -v || echo "[WARN] sudo authentication failed"


# ============================================================
# 1. CAN + EtherCAT
# ============================================================

HARDWARE_LOG="${LOG_DIR}/hardware.log"

echo "[1/7] CAN + EtherCAT"
echo "      log: ${HARDWARE_LOG}"

{
    echo "============================================================"
    echo "CAN"
    echo "time: $(date)"
    echo "interface: ${CAN_IFACE}"
    echo "bitrate: ${CAN_BITRATE}"
    echo "============================================================"

    sudo_with_password ip link set "${CAN_IFACE}" down 2>/dev/null || true

    sudo_with_password ip link set "${CAN_IFACE}" up type can bitrate "${CAN_BITRATE}" || \
        echo "[WARN] CAN startup failed"

    echo
    ip -details link show "${CAN_IFACE}" 2>&1 || true

    echo
    echo "============================================================"
    echo "EtherCAT"
    echo "============================================================"

    if command -v ethercatctl >/dev/null 2>&1; then
        sudo_with_password ethercatctl start || \
            echo "[WARN] ethercatctl start failed"

        sudo_with_password ethercatctl status 2>&1 || true

    elif command -v ethercat >/dev/null 2>&1; then
        sudo_with_password ethercat start || \
            echo "[WARN] ethercat start failed"

        sudo_with_password ethercat slaves 2>&1 || true

    else
        echo "[WARN] ethercat command not found"
    fi
} >"${HARDWARE_LOG}" 2>&1

# EtherCAT master 启动后，不立即启动 Taihu。
# 等待总线上恰好出现 4 个转向电机从站，避免 Taihu 在 PDO 尚未注册时启动。
if wait_taihu_slaves \
    "${TAIHU_EXPECTED_SLAVES}" \
    "${TAIHU_SLAVE_WAIT_SEC}" \
    >>"${HARDWARE_LOG}" 2>&1
then
    TAIHU_ETHERCAT_READY=1
else
    TAIHU_ETHERCAT_READY=0
fi


# ============================================================
# 2. ZLAC + Taihu
# ============================================================

echo "[2/7] chassis"

start_user \
    "ZLAC four-wheel driver" \
    "${LOG_DIR}/zlac.log" \
    ros2 launch \
        zlac8015d_four_wheel_driver_cpp \
        four_wheel_driver.launch.py

if [ "${TAIHU_ETHERCAT_READY}" -eq 1 ]; then
    start_root_taihu
else
    echo "[SKIP][root] Taihu steer driver"
    echo "             reason: EtherCAT steering slave count != ${TAIHU_EXPECTED_SLAVES}"
    {
        echo "[ERROR] Taihu driver was not started."
        echo "Expected EtherCAT steering slaves: ${TAIHU_EXPECTED_SLAVES}"
        echo "See ${HARDWARE_LOG} for 'ethercat slaves' output."
    } >"${LOG_DIR}/taihu.log"
fi


# ============================================================
# 3. 电机 clear fault / enable
#
# 后台执行，不阻塞其它模块启动。
# 结果全部记录到 motor_enable.log。
# ============================================================

echo "[3/7] motor enable"
echo "      log: ${LOG_DIR}/motor_enable.log"

(
    {
        echo "============================================================"
        echo "Motor enable"
        echo "time: $(date)"
        echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
        echo "============================================================"

        # ----------------------------------------------------
        # Trigger service：不仅“尝试调用”，还检查 success=True。
        # ----------------------------------------------------
        call_trigger_success()
        {
            local service="$1"
            local attempts="${2:-6}"
            local timeout_sec="${3:-8}"
            local i
            local output
            local rc

            for ((i=1; i<=attempts; ++i)); do
                echo
                echo "[CALL ${i}/${attempts}] ${service}"
                echo "[DDS] DOMAIN=${ROS_DOMAIN_ID} TRANSPORT=${FASTDDS_BUILTIN_TRANSPORTS}"

                output="$(
                    ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
                    ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
                    FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
                    timeout --signal=INT "${timeout_sec}" \
                    ros2 service call \
                        "${service}" \
                        std_srvs/srv/Trigger "{}" 2>&1
                )"

                rc=$?

                echo "${output}"

                if [ ${rc} -eq 0 ] &&
                echo "${output}" | grep -q "success=True"
                then
                    echo "[SERVICE OK] ${service}: success=True"
                    return 0
                fi

                echo "[WARN] ${service}: no success=True, retrying..."
                sleep 1
            done

            echo "[ERROR] ${service}: service did not report success=True"
            return 1
        }

        # ----------------------------------------------------
        # ZLAC 实际状态确认
        #
        # /enable 返回 success=True 后，继续观察 zlac.log。
        # 驱动内部未使能时会周期性打印：
        #   drivers are not enabled; call /enable first
        #
        # 所以记录 enable 前日志行号，只检查 enable 后新增内容。
        # ----------------------------------------------------
        verify_zlac_enabled()
        {
            local logfile="${LOG_DIR}/zlac.log"
            local start_line="$1"
            local timeout_sec="${2:-5}"
            local i

            for ((i=1; i<=timeout_sec; ++i)); do
                sleep 1

                if tail -n +"${start_line}" "${logfile}" 2>/dev/null |
                   grep -q "drivers are not enabled; call /enable first"
                then
                    echo "[VERIFY] ZLAC still reports NOT enabled (${i}/${timeout_sec}s)"
                    continue
                fi

                # 至少等 2 秒，跨过驱动原有约 2 秒一次的 warning 周期，
                # 再判定没有新的 not-enabled warning。
                if [ ${i} -ge 3 ]; then
                    echo "[VERIFY OK] ZLAC no longer reports 'drivers are not enabled'"
                    return 0
                fi
            done

            echo "[VERIFY FAILED] ZLAC still not enabled"
            return 1
        }

        # ----------------------------------------------------
        # Taihu 实际状态确认
        #
        # /steer/enable 的 Trigger response 是 "enable requested"，
        # 只表示请求被接受。
        # 真正的硬件状态以 EtherCAT 周期日志：
        #   enabled=[1, 1, 1, 1]
        # 为准。
        # ----------------------------------------------------
        verify_taihu_enabled()
        {
            local logfile="${LOG_DIR}/taihu.log"
            local timeout_sec="${1:-8}"
            local i

            for ((i=1; i<=timeout_sec; ++i)); do
                if tail -n 80 "${logfile}" 2>/dev/null |
                   grep -q "enabled=\[1, 1, 1, 1\]"
                then
                    echo "[VERIFY OK] Taihu enabled=[1, 1, 1, 1]"
                    return 0
                fi

                echo "[VERIFY] waiting Taihu actual enable (${i}/${timeout_sec}s)"
                sleep 1
            done

            echo "[VERIFY FAILED] Taihu did not reach enabled=[1, 1, 1, 1]"
            return 1
        }

        # ZLAC 启动较快，先处理驱动轮。
        sleep 2

        call_trigger_success \
            /wheel_control_can/clear_fault 6 3 || true

        zlac_log_start=$(( $(wc -l < "${LOG_DIR}/zlac.log" 2>/dev/null || echo 0) + 1 ))

        if call_trigger_success \
            /wheel_control_can/enable 6 3
        then
            verify_zlac_enabled "${zlac_log_start}" 5 || true
        else
            echo "[VERIFY SKIP] ZLAC enable service never returned success=True"
        fi

        # Taihu 只有在 EtherCAT 检测到恰好 4 个转向电机从站时才会启动。
        if [ "${TAIHU_ETHERCAT_READY}" -eq 1 ]; then
            # Taihu 启动后还会依次读取 4 个 EtherCAT 从站的 PP 参数。
            sleep 6

            call_trigger_success \
                /steer/reset_fault 6 8 || true

            if call_trigger_success \
                /steer/enable 6 8
            then
                verify_taihu_enabled 8 || true
            else
                echo "[VERIFY SKIP] Taihu enable service never returned success=True"
            fi
        else
            echo "[VERIFY SKIP] Taihu EtherCAT slave count check failed; Taihu was not started"
        fi

    } >"${LOG_DIR}/motor_enable.log" 2>&1
) &


# ============================================================
# 4. Livox
# ============================================================

echo "[4/7] Livox"

start_user \
    "Livox MID360" \
    "${LOG_DIR}/livox.log" \
    ros2 launch \
        livox_ros_driver2 \
        msg_MID360s_launch.py


# ============================================================
# 5. FAST-LIO + Open3D + System Manager
# ============================================================

echo "[5/7] FAST-LIO + Open3D"

start_user \
    "FAST-LIO / Open3D" \
    "${LOG_DIR}/localization.log" \
    ros2 launch \
        open3d_loc \
        localization_3d_g1.launch.py

start_user \
    "System Manager" \
    "${LOG_DIR}/system_manager.log" \
    ros2 launch \
        system_manager \
        system_manager.launch.py


# ============================================================
# 6. g1_nav2
# ============================================================

echo "[6/7] g1_nav2"

# start_user \
#     "G1 Swerve Nav2" \
#     "${LOG_DIR}/g1_swerve_nav.log" \
#     ros2 launch \
#         g1_swerve_nav \
#         bringup.launch.py



start_user \
    "ackermann" \
    "${LOG_DIR}/ackermann.log" \
    ros2 launch \
        ackermann_steering_controller \
        ackermann_standalone.launch.py

start_user \
    "Nav2" \
    "${LOG_DIR}/nav2.log" \
    ros2 launch \
        nav2 \
        swerve_navigation2_gazebo.launch.py


# ============================================================
# 7. merman_robot
# ============================================================

echo "[7/7] merman_robot"

start_user \
    "Merman Robot" \
    "${LOG_DIR}/merman_robot.log" \
    "${MERMAN_ROBOT}"


# ============================================================
# 完成
# ============================================================

echo
echo "============================================================"
echo " All launch commands issued."
echo "============================================================"
echo
echo "日志目录："
echo "  ${LOG_DIR}"
echo
echo "快捷路径："
echo "  ${LOG_ROOT}/latest"
echo
echo "查看各模块："
echo "  tail -f ${LOG_ROOT}/latest/zlac.log"
echo "  tail -f ${LOG_ROOT}/latest/taihu.log"
echo "  tail -f ${LOG_ROOT}/latest/motor_enable.log"
echo "  tail -f ${LOG_ROOT}/latest/livox.log"
echo "  tail -f ${LOG_ROOT}/latest/localization.log"
echo "  tail -f ${LOG_ROOT}/latest/system_manager.log"
echo "  tail -f ${LOG_ROOT}/latest/g1_nav2.log"
echo
echo "快速找错误："
echo "  grep -RniE 'FATAL|ERROR|process has died|exception|failed' ${LOG_ROOT}/latest"
echo
echo "Ctrl+C stop."
echo


# ============================================================
# 前台只保持 bringup 生命周期，不打印子节点日志
# ============================================================

while true; do
    sleep 10
done
