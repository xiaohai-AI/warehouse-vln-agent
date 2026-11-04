# ground_truth/poses.csv

真值轨迹。由 `python3 scripts/record_gt_poses.py`（订阅 /world/depot/pose/info 里的
depot_bot 位姿）在仿真运行时写入，20 Hz。

列：t(秒, sim time) x y z qx qy qz qw vx vy wz
本文件只用于 ATE/RPE 打分，**运行时禁止被 agent 读取**。
