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
CAN_READY_WAIT_SEC="${CAN_READY_WAIT_SEC:-15}"
CAN_READY=0

# Taihu EtherCAT bus is expected to contain exactly four steering motor slaves.
TAIHU_EXPECTED_SLAVES="${TAIHU_EXPECTED_SLAVES:-4}"
TAIHU_MASTER_WAIT_SEC="${TAIHU_MASTER_WAIT_SEC:-15}"
TAIHU_SLAVE_WAIT_SEC="${TAIHU_SLAVE_WAIT_SEC:-15}"
TAIHU_ETHERCAT_MASTER_READY=0
TAIHU_ETHERCAT_READY=0

# Motor services are created asynchronously by the two driver nodes.  Do not
# fail only because a service appears a few seconds late: wait and retry until
# the motor is positively verified enabled.
MOTOR_SERVICE_POLL_SEC="${MOTOR_SERVICE_POLL_SEC:-1}"
MOTOR_ENABLE_RETRY_SEC="${MOTOR_ENABLE_RETRY_SEC:-2}"
ZLAC_ENABLE_VERIFY_SEC="${ZLAC_ENABLE_VERIFY_SEC:-5}"
TAIHU_ENABLE_VERIFY_SEC="${TAIHU_ENABLE_VERIFY_SEC:-10}"

# ============================================================
# CPU affinity
#
# CPU0   : OS / interrupts / DDS daemons / background programs
# CPU1   : EtherCAT realtime thread (set by Taihu cpu_affinity)
# CPU2-4 : Livox / FAST-LIO / Open3D localization
# CPU5   : chassis / command smoothing / collision monitoring
# CPU6   : controller_server (MPPI + local costmap)
# CPU7   : planner / behavior server / behavior tree
#
# These defaults can be overridden from the environment for diagnostics.
# Keep CPU1 out of every group below: it is reserved for EtherCAT.
# ============================================================
CPU_BACKGROUND="${CPU_BACKGROUND:-0}"
CPU_LOCALIZATION="${CPU_LOCALIZATION:-2-4}"
CPU_CHASSIS="${CPU_CHASSIS:-5}"
CPU_CONTROLLER="${CPU_CONTROLLER:-6}"
CPU_PLANNING="${CPU_PLANNING:-7}"
CPU_NAV_INITIAL="${CPU_NAV_INITIAL:-5-7}"

# The per-node ROS launch prefixes consume these values. Export them so each
# node starts on its final CPU set and all of its threads inherit that set.
export CPU_CHASSIS CPU_CONTROLLER CPU_PLANNING

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
    local cpus="$1"
    local name="$2"
    local logfile="$3"
    shift 3

    echo "[START] ${name}"
    echo "        log: ${logfile}"
    echo "        CPU: ${cpus}"

    # 每个 ros2 launch 的 stdout + stderr 全部进入自己的日志文件。
    taskset --cpu-list "${cpus}" "$@" >"${logfile}" 2>&1 &

    local pid=$!
    add_pid "${pid}" "${name}"

    echo "        pid: ${pid}"
}


check_cpu_affinity()
{
    local cpus

    if ! command -v taskset >/dev/null 2>&1; then
        echo "[ERROR] taskset not found; CPU affinity cannot be applied"
        exit 1
    fi

    for cpus in \
        "${CPU_BACKGROUND}" \
        "${CPU_LOCALIZATION}" \
        "${CPU_CHASSIS}" \
        "${CPU_CONTROLLER}" \
        "${CPU_PLANNING}" \
        "${CPU_NAV_INITIAL}"
    do
        if ! taskset --cpu-list "${cpus}" true >/dev/null 2>&1; then
            echo "[ERROR] CPU list '${cpus}' is unavailable or disallowed"
            exit 1
        fi
    done

    # Prevent the bringup shell and its unclassified/background children from
    # ever running on CPU1. Explicitly grouped children are moved below.
    if ! taskset --pid --cpu-list "${CPU_BACKGROUND}" "$$" >/dev/null 2>&1; then
        echo "[ERROR] failed to pin bringup shell to CPU${CPU_BACKGROUND}"
        exit 1
    fi
}


wait_can_ready()
{
    local timeout_sec="${1:-15}"
    local i
    local output

    echo "============================================================"
    echo "CAN readiness check"
    echo "interface: ${CAN_IFACE}"
    echo "expected bitrate: ${CAN_BITRATE}"
    echo "timeout: ${timeout_sec}s"
    echo "============================================================"

    for ((i=1; i<=timeout_sec; ++i)); do
        output="$(ip -details link show "${CAN_IFACE}" 2>&1)"

        echo
        echo "[CHECK ${i}/${timeout_sec}] CAN ${CAN_IFACE}"
        echo "${output}"

        if printf '%s\n' "${output}" | grep -qE '<[^>]*UP[^>]*LOWER_UP[^>]*>' &&
           printf '%s\n' "${output}" | grep -qE 'state[[:space:]]+UP' &&
           printf '%s\n' "${output}" | grep -qE 'can state[[:space:]]+ERROR-ACTIVE' &&
           printf '%s\n' "${output}" | grep -qE "bitrate[[:space:]]+${CAN_BITRATE}([[:space:]]|$)"
        then
            echo "[OK] CAN ${CAN_IFACE} is UP / LOWER_UP / ERROR-ACTIVE @ ${CAN_BITRATE}"
            return 0
        fi

        sleep 1
    done

    echo "[ERROR] CAN ${CAN_IFACE} did not become healthy"
    return 1
}


wait_ethercat_master()
{
    local timeout_sec="${1:-15}"
    local i
    local output
    local rc

    echo "============================================================"
    echo "EtherCAT master readiness check"
    echo "timeout: ${timeout_sec}s"
    echo "============================================================"

    for ((i=1; i<=timeout_sec; ++i)); do
        if command -v ethercatctl >/dev/null 2>&1; then
            output="$(sudo_with_password ethercatctl status 2>&1)"
            rc=$?

            echo
            echo "[CHECK ${i}/${timeout_sec}] EtherCAT master"
            echo "${output}"

            if [ ${rc} -eq 0 ] &&
               printf '%s\n' "${output}" | grep -qE 'Master[0-9]+[[:space:]]+running'
            then
                echo "[OK] EtherCAT master is running"
                return 0
            fi
        elif command -v ethercat >/dev/null 2>&1; then
            # On systems without ethercatctl, a successful master query proves
            # that the IgH master device is present and accessible.  The exact
            # steering-bus health is confirmed immediately afterwards by the
            # four-slave check.
            output="$(sudo_with_password ethercat master 2>&1)"
            rc=$?

            echo
            echo "[CHECK ${i}/${timeout_sec}] EtherCAT master"
            echo "${output}"

            if [ ${rc} -eq 0 ] && [ -n "${output}" ]; then
                echo "[OK] EtherCAT master query succeeded"
                return 0
            fi
        else
            echo "[ERROR] neither ethercatctl nor ethercat CLI is available"
            return 1
        fi

        sleep 1
    done

    echo "[ERROR] EtherCAT master did not become ready"
    return 1
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
        taskset --cpu-list "${CPU_CHASSIS}" \
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
echo "CPU0 background           = ${CPU_BACKGROUND}"
echo "CPU1 EtherCAT RT          = 1 (Taihu parameter)"
echo "CPU2-4 localization       = ${CPU_LOCALIZATION}"
echo "CPU5 chassis/safety       = ${CPU_CHASSIS}"
echo "CPU6 controller/MPPI      = ${CPU_CONTROLLER}"
echo "CPU7 planner/behavior/BT  = ${CPU_PLANNING}"
echo "============================================================"

check_cpu_affinity

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

# CAN 必须达到与正常 hardware.log 一致的健康状态：
#   interface UP + LOWER_UP, state UP, can state ERROR-ACTIVE, bitrate 正确。
if wait_can_ready "${CAN_READY_WAIT_SEC}" >>"${HARDWARE_LOG}" 2>&1; then
    CAN_READY=1
else
    CAN_READY=0
fi

# EtherCAT 先确认 master running，再等待总线上恰好出现 4 个泰虎机电从站。
if wait_ethercat_master "${TAIHU_MASTER_WAIT_SEC}" >>"${HARDWARE_LOG}" 2>&1; then
    TAIHU_ETHERCAT_MASTER_READY=1
else
    TAIHU_ETHERCAT_MASTER_READY=0
fi

if [ "${TAIHU_ETHERCAT_MASTER_READY}" -eq 1 ] &&
   wait_taihu_slaves \
       "${TAIHU_EXPECTED_SLAVES}" \
       "${TAIHU_SLAVE_WAIT_SEC}" \
       >>"${HARDWARE_LOG}" 2>&1
then
    TAIHU_ETHERCAT_READY=1
else
    TAIHU_ETHERCAT_READY=0
fi

{
    echo
    echo "============================================================"
    echo "Hardware readiness summary"
    echo "CAN_READY=${CAN_READY}"
    echo "TAIHU_ETHERCAT_MASTER_READY=${TAIHU_ETHERCAT_MASTER_READY}"
    echo "TAIHU_ETHERCAT_READY=${TAIHU_ETHERCAT_READY}"
    echo "============================================================"
} >>"${HARDWARE_LOG}" 2>&1


# ============================================================
# 2. ZLAC + Taihu
# ============================================================

echo "[2/7] chassis"

if [ "${CAN_READY}" -eq 1 ]; then
    start_user \
        "${CPU_CHASSIS}" \
        "ZLAC four-wheel driver" \
        "${LOG_DIR}/zlac.log" \
        ros2 launch \
            zlac8015d_four_wheel_driver_cpp \
            four_wheel_driver.launch.py
else
    echo "[SKIP] ZLAC four-wheel driver"
    echo "       reason: CAN ${CAN_IFACE} is not healthy"
    {
        echo "[ERROR] ZLAC driver was not started."
        echo "CAN interface ${CAN_IFACE} failed the readiness check."
        echo "See ${HARDWARE_LOG}."
    } >"${LOG_DIR}/zlac.log"
fi

if [ "${TAIHU_ETHERCAT_READY}" -eq 1 ]; then
    start_root_taihu
else
    echo "[SKIP][root] Taihu steer driver"
    echo "             reason: EtherCAT master/slave readiness check failed"
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
# 不再使用固定 sleep 后只尝试几次：
#   1) 等待 Trigger service 真正出现；
#   2) clear fault；
#   3) enable；
#   4) 验证实际使能状态；
#   5) 任一步失败就继续循环，直到验证成功。
# ============================================================

echo "[3/7] motor enable"
echo "      log: ${LOG_DIR}/motor_enable.log"

(
    {
        taskset --pid --cpu-list "${CPU_CHASSIS}" "${BASHPID}" >/dev/null 2>&1 || \
            echo "[WARN] failed to pin motor-enable helper to CPU${CPU_CHASSIS}"

        echo "============================================================"
        echo "Motor enable"
        echo "time: $(date)"
        echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
        echo "CAN_READY=${CAN_READY}"
        echo "TAIHU_ETHERCAT_READY=${TAIHU_ETHERCAT_READY}"
        echo "============================================================"

        service_is_trigger()
        {
            local service="$1"
            local type

            type="$(timeout --signal=INT 4 \
                env \
                    ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
                    ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
                    FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
                    ros2 service type "${service}" 2>/dev/null | tail -n 1)"

            [ "${type}" = "std_srvs/srv/Trigger" ]
        }

        wait_for_trigger_service()
        {
            local service="$1"
            local n=0

            while true; do
                if service_is_trigger "${service}"; then
                    echo "[SERVICE READY] ${service}"
                    return 0
                fi

                n=$((n + 1))
                echo "[WAIT ${n}] ${service} not available yet; waiting..."
                sleep "${MOTOR_SERVICE_POLL_SEC}"
            done
        }

        call_trigger_once()
        {
            local service="$1"
            local timeout_sec="${2:-8}"
            local output
            local rc

            echo "[CALL] ${service}"
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
               printf '%s\n' "${output}" | grep -q 'success=True'
            then
                echo "[SERVICE OK] ${service}: success=True"
                return 0
            fi

            echo "[SERVICE FAILED] ${service}: rc=${rc}, no success=True"
            return 1
        }

        verify_zlac_enabled()
        {
            local logfile="${LOG_DIR}/zlac.log"
            local verify_sec="${1:-5}"
            local pattern='drivers are not enabled; call /enable first'
            local before
            local after

            before="$(grep -cF "${pattern}" "${logfile}" 2>/dev/null || true)"
            before="${before:-0}"

            echo "[VERIFY] ZLAC: watch ${verify_sec}s for new NOT-enabled warnings (baseline=${before})"
            sleep "${verify_sec}"

            after="$(grep -cF "${pattern}" "${logfile}" 2>/dev/null || true)"
            after="${after:-0}"

            if [ "${after}" -eq "${before}" ]; then
                echo "[VERIFY OK] ZLAC enabled: no new NOT-enabled warning (${before} -> ${after})"
                return 0
            fi

            echo "[VERIFY FAILED] ZLAC still reports NOT enabled (${before} -> ${after})"
            return 1
        }

        verify_taihu_enabled()
        {
            local logfile="${LOG_DIR}/taihu.log"
            local timeout_sec="${1:-10}"
            local i
            local last_state

            for ((i=1; i<=timeout_sec; ++i)); do
                last_state="$(
                    grep -oE 'enabled=\[[01], [01], [01], [01]\]' "${logfile}" 2>/dev/null |
                    tail -n 1
                )"

                if [ "${last_state}" = "enabled=[1, 1, 1, 1]" ]; then
                    echo "[VERIFY OK] Taihu ${last_state}"
                    return 0
                fi

                echo "[VERIFY ${i}/${timeout_sec}] Taihu latest state: ${last_state:-not reported yet}"
                sleep 1
            done

            echo "[VERIFY FAILED] Taihu latest state did not reach enabled=[1, 1, 1, 1]"
            return 1
        }

        enable_zlac_until_success()
        {
            local round=0

            wait_for_trigger_service /wheel_control_can/clear_fault
            wait_for_trigger_service /wheel_control_can/enable

            while true; do
                round=$((round + 1))
                echo
                echo "============================================================"
                echo "[ZLAC ENABLE ROUND ${round}]"
                echo "============================================================"

                # Services can disappear when the driver is restarted.  Re-check
                # them on every round instead of assuming they stay alive.
                wait_for_trigger_service /wheel_control_can/clear_fault
                wait_for_trigger_service /wheel_control_can/enable

                if ! call_trigger_once /wheel_control_can/clear_fault 5; then
                    echo "[ZLAC] clear_fault failed; retry in ${MOTOR_ENABLE_RETRY_SEC}s"
                    sleep "${MOTOR_ENABLE_RETRY_SEC}"
                    continue
                fi

                if ! call_trigger_once /wheel_control_can/enable 5; then
                    echo "[ZLAC] enable request failed; retry in ${MOTOR_ENABLE_RETRY_SEC}s"
                    sleep "${MOTOR_ENABLE_RETRY_SEC}"
                    continue
                fi

                if verify_zlac_enabled "${ZLAC_ENABLE_VERIFY_SEC}"; then
                    echo "[MOTOR READY] ZLAC drive motors are enabled"
                    return 0
                fi

                echo "[ZLAC] enable was not confirmed; clear fault + enable again in ${MOTOR_ENABLE_RETRY_SEC}s"
                sleep "${MOTOR_ENABLE_RETRY_SEC}"
            done
        }

        enable_taihu_until_success()
        {
            local round=0

            wait_for_trigger_service /steer/reset_fault
            wait_for_trigger_service /steer/enable

            while true; do
                round=$((round + 1))
                echo
                echo "============================================================"
                echo "[TAIHU ENABLE ROUND ${round}]"
                echo "============================================================"

                wait_for_trigger_service /steer/reset_fault
                wait_for_trigger_service /steer/enable

                if ! call_trigger_once /steer/reset_fault 8; then
                    echo "[TAIHU] reset_fault failed; retry in ${MOTOR_ENABLE_RETRY_SEC}s"
                    sleep "${MOTOR_ENABLE_RETRY_SEC}"
                    continue
                fi

                if ! call_trigger_once /steer/enable 8; then
                    echo "[TAIHU] enable request failed; retry in ${MOTOR_ENABLE_RETRY_SEC}s"
                    sleep "${MOTOR_ENABLE_RETRY_SEC}"
                    continue
                fi

                if verify_taihu_enabled "${TAIHU_ENABLE_VERIFY_SEC}"; then
                    echo "[MOTOR READY] Taihu steering motors are enabled"
                    return 0
                fi

                echo "[TAIHU] enable was not confirmed; reset fault + enable again in ${MOTOR_ENABLE_RETRY_SEC}s"
                sleep "${MOTOR_ENABLE_RETRY_SEC}"
            done
        }

        # 两套电机分别处理。硬件接口不正常时不向对应总线下发电机命令。
        # 这里本身运行在后台，因此等待 service / 重试使能不会卡住 Livox、
        # FAST-LIO、Nav2 等后续模块的启动。
        if [ "${CAN_READY}" -eq 1 ]; then
            enable_zlac_until_success &
            ZLAC_ENABLE_PID=$!
        else
            echo "[SKIP] ZLAC enable: CAN ${CAN_IFACE} is not healthy; see ${HARDWARE_LOG}"
            ZLAC_ENABLE_PID=""
        fi

        if [ "${TAIHU_ETHERCAT_READY}" -eq 1 ]; then
            enable_taihu_until_success &
            TAIHU_ENABLE_PID=$!
        else
            echo "[SKIP] Taihu enable: EtherCAT is not healthy; see ${HARDWARE_LOG}"
            TAIHU_ENABLE_PID=""
        fi

        # Keep this logging subshell alive until both enable workers finish.
        # In the normal case they exit only after positive verification.
        [ -n "${ZLAC_ENABLE_PID}" ] && wait "${ZLAC_ENABLE_PID}"
        [ -n "${TAIHU_ENABLE_PID}" ] && wait "${TAIHU_ENABLE_PID}"

        echo
        echo "============================================================"
        echo "Motor enable complete"
        echo "============================================================"

    } >"${LOG_DIR}/motor_enable.log" 2>&1
) &


# ============================================================
# 4. Livox
# ============================================================

echo "[4/7] Livox"

start_user \
    "${CPU_LOCALIZATION}" \
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
    "${CPU_LOCALIZATION}" \
    "FAST-LIO / Open3D" \
    "${LOG_DIR}/localization.log" \
    ros2 launch \
        open3d_loc \
        localization_3d_g1.launch.py

start_user \
    "${CPU_BACKGROUND}" \
    "System Manager" \
    "${LOG_DIR}/system_manager.log" \
    ros2 launch \
        system_manager \
        system_manager.launch.py


# ============================================================
# 6. g1_nav2
# ============================================================

echo "[6/7] g1_nav2"

start_user \
    "${CPU_NAV_INITIAL}" \
    "G1 Swerve Nav2" \
    "${LOG_DIR}/g1_swerve_nav.log" \
    ros2 launch \
        g1_swerve_nav \
        bringup.launch.py



# start_user \
#     "ackermann" \
#     "${LOG_DIR}/ackermann.log" \
#     ros2 launch \
#         ackermann_steering_controller \
#         ackermann_standalone.launch.py

# start_user \
#     "Nav2" \
#     "${LOG_DIR}/nav2.log" \
#     ros2 launch \
#         nav2 \
#         swerve_navigation2_gazebo.launch.py


# ============================================================
# 7. merman_robot
# ============================================================

echo "[7/7] merman_robot"

start_user \
    "${CPU_BACKGROUND}" \
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