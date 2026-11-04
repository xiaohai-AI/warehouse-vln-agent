#!/usr/bin/env bash
# 一条命令拆干净整套栈。每次重起前必须先跑 —— 我们踩过"两个 gz server 并存"：
# 每个 server 都持有独立的 GL/EGL 上下文，叠起来会把 nvidia-modeset 挂死（桌面冻结）。
# 注意：pgrep/pkill 的模式必须方括号化（如 [w]arehouse），否则匹配到自己 shell 会自杀。
set +e

pkill -f "[w]arehouse.sdf"
pkill -f "[d]epot_sim.launch.py"
pkill -f "[p]arameter_bridge"
pkill -f "[m]ecanum_odom.py"
pkill -f "[r]obot_state_publisher"
pkill -f "[p]ointcloud_to_laserscan"
pkill -f "[s]lam_toolbox"
pkill -f "[n]av2_"
pkill -f "[l]ifecycle_manager"
pkill -f "[b]t_navigator"
pkill -f "[c]ontroller_server"
pkill -f "[p]lanner_server"
pkill -f "[s]moother_server"
pkill -f "[b]ehavior_server"
pkill -f "[w]aypoint_follower"
pkill -f "[v]elocity_smoother"
pkill -f "[s]tatic_map_pub"
pkill -f "[t]ruth_localization"
pkill -f "[g]pu_watchdog"

sleep 5
# 兜底：gz server 关渲染上下文时经常不理 SIGTERM（实测能拖很久），这里补一轮 SIGKILL。
# 不补的话 `make down` 会误报失败，而且残留的 server 会和下一次启动叠成两个实例。
pkill -9 -f "[w]arehouse.sdf"
pkill -9 -f "[d]epot_sim.launch.py"
pkill -9 -f "[p]arameter_bridge"
pkill -9 -f "[m]ecanum_odom.py"
pkill -9 -f "[r]obot_state_publisher"
pkill -9 -f "[s]lam_toolbox"
pkill -9 -f "[n]av2_"
pkill -9 -f "[s]tatic_map_pub"
pkill -9 -f "[t]ruth_localization"
pkill -9 -f "[g]pu_watchdog"
sleep 2
left=$(pgrep -af "[w]arehouse.sdf|[d]epot_sim.launch|[p]arameter_bridge|[m]ecanum_odom|[n]av2_|[s]lam_toolbox|[s]tatic_map_pub|[t]ruth_localization" 2>/dev/null | grep -vc stack_down)
echo "已拆除；关键进程残留数 = $left"
if [ "$left" -ne 0 ]; then
  pgrep -af "[w]arehouse.sdf|[d]epot_sim.launch|[p]arameter_bridge|[m]ecanum_odom|[n]av2_|[s]lam_toolbox" | grep -v stack_down | head -8
  exit 1
fi
