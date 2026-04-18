#!/usr/bin/env bash
set -u

# hero_aim 看门狗启动脚本。

# 检测是否在终端中运行，如果不是，在新终端中启动自己
if [[ ! -t 0 ]] || [[ "${IN_NEW_TERMINAL:-}" != "1" ]]; then
    SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
    export IN_NEW_TERMINAL=1
    gnome-terminal -- bash -c "cd ${PROJECT_ROOT} && ./scripts/auto_start.sh $*; exec bash"
    exit 0
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

# 路径与参数配置（按需修改）。
BIN_REL="./build/hero"
CFG_REL="configs/hero.yaml"
LOG_FILE="${PROJECT_ROOT}/logs/hero.log"
PID_FILE="${PROJECT_ROOT}/logs/hero.pid"
RESTART_DELAY_SEC=2
RUN_TIMEOUT_SEC=0

BIN_PATH="${PROJECT_ROOT}/${BIN_REL}"
CFG_PATH="${PROJECT_ROOT}/${CFG_REL}"

mkdir -p "${PROJECT_ROOT}/logs"

# 确保相对资源路径（如 assets/*.onnx）解析正确。
cd "${PROJECT_ROOT}"

# 记录看门狗 PID，便于手动停止或排查。
ACTION="${1:-start}"
CHILD_PID=""

is_running() {
  if [[ -f "${PID_FILE}" ]]; then
    read -r pid < "${PID_FILE}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      echo "${pid}"
      return 0
    fi
  fi
  return 1
}

stop_watchdog() {
  if pid=$(is_running); then
    kill "${pid}" 2>/dev/null || true
    sleep 1
    kill -9 "${pid}" 2>/dev/null || true
    rm -f "${PID_FILE}"
    echo "[$(date -Is)] launcher stop" >> "${LOG_FILE}"
    echo "已停止，PID=${pid}"
  else
    echo "未在运行"
  fi
}

status_watchdog() {
  if pid=$(is_running); then
    echo "运行中，PID=${pid}"
    return 0
  fi
  echo "未在运行"
  return 1
}

case "${ACTION}" in
  start)
    if is_running >/dev/null; then
      echo "已在运行，PID=$(is_running)"
      exit 0
    fi
    ;;
  stop)
    stop_watchdog
    exit 0
    ;;
  status)
    status_watchdog
    exit $?
    ;;
  restart)
    stop_watchdog
    ;;
  *)
    echo "用法: $0 {start|stop|status|restart}"
    exit 2
    ;;
esac

echo $$ > "${PID_FILE}"

cleanup() {
  if [[ -n "${CHILD_PID}" ]] && kill -0 "${CHILD_PID}" 2>/dev/null; then
    kill "${CHILD_PID}" 2>/dev/null || true
    sleep 1
    kill -9 "${CHILD_PID}" 2>/dev/null || true
  fi
  rm -f "${PID_FILE}"
  echo "[$(date -Is)] launcher exit" >> "${LOG_FILE}"
  exit 0
}

trap cleanup INT TERM

echo "[$(date -Is)] launcher start" >> "${LOG_FILE}"

while true; do
  # 二进制不存在或不可执行时，等待后重试。
  if [[ ! -x "${BIN_PATH}" ]]; then
    echo "[$(date -Is)] binary not found or not executable: ${BIN_PATH}" >> "${LOG_FILE}"
    sleep "${RESTART_DELAY_SEC}"
    continue
  fi

  echo "[$(date -Is)] starting: ${BIN_PATH} ${CFG_PATH}" >> "${LOG_FILE}"

  # 超时未退出则强制终止，避免卡死；设为 0 则不启用超时。
  # 使用 tee 同时输出到终端和日志文件，便于排查串口/相机掉线等问题。
  if [[ "${RUN_TIMEOUT_SEC}" -gt 0 ]]; then
    timeout --signal=TERM --kill-after=5 "${RUN_TIMEOUT_SEC}" \
      "${BIN_PATH}" "${CFG_PATH}" 2>&1 | tee -a "${LOG_FILE}" &
    CHILD_PID=$!
    wait "${CHILD_PID}"
    EXIT_CODE=$?
  else
    "${BIN_PATH}" "${CFG_PATH}" 2>&1 | tee -a "${LOG_FILE}" &
    CHILD_PID=$!
    wait "${CHILD_PID}"
    EXIT_CODE=$?
  fi

  # 进程退出后立即重启，形成看门狗效果。
  if [[ ${EXIT_CODE} -eq 124 || ${EXIT_CODE} -eq 137 ]]; then
    echo "[$(date -Is)] 超时退出(${RUN_TIMEOUT_SEC}s)，${RESTART_DELAY_SEC}s后重启" >> "${LOG_FILE}"
  else
    echo "[$(date -Is)] exit code: ${EXIT_CODE}, restarting in ${RESTART_DELAY_SEC}s" >> "${LOG_FILE}"
  fi
  sleep "${RESTART_DELAY_SEC}"
done
