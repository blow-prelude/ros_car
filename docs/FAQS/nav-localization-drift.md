# 导航定位类

## 跑几个航点后"地图和定位漂移"、车在门洞附近原地打转
- **现象**：`navigation.launch.py` 起导航，前几个航点正常；之后 RViz 里机器人位姿相对地图
  来回跳动，局部代价地图/雷达点云相对全局地图慢慢转开，车卡在门洞一侧小范围徘徊，
  `controller_server` 连刷 `Failed to make progress`，后续航点 240s 超时失败。
- **原因**（2026-10-15 航点回归 + gz 真值对账实测）：链条是三层——
  1. 打滑使轮速里程计 yaw 系统性偏大（原地 ~15%，混合行驶 ~2.5%/rad），且随**累计旋转量**
     线性累积（DWB 转向 + BT 恢复行为都在贡献旋转，一轮回归累计 ~44-64 rad）；
  2. 旧 `ekf.yaml` 同时融合轮速绝对 x/y/yaw 与 IMU 绝对 yaw，而 diff_drive 插件的
     `covariance_yaw`=0.001 比 imu_filter 的 `orientation_covariance`=0.01 小一个量级，
     Kalman 权重 ~91% 给了**更不准的**轮速——64 rad 旋转后 `odom->base_footprint` 偏航误差
     涨到 1.5 rad（IMU 输入本身与真值偏差 <0.04 rad，被权重淹没）；
  3. 下游全被带崩：AMCL 只能用 `map->odom` 反向硬拉（累计 -1.4 rad），每次粒子更新跳一步
     → RViz 里"定位漂"；局部代价地图在 odom 系里按错误偏航插入雷达点 → 障碍涂抹成幻影墙
     封住门洞 → 车打转 → 恢复行为更转 → 误差更大，恶性循环 → "地图漂"。
- **解决**：`car_localization/config/ekf.yaml` 轮速里程计只融合 `vx`+`vyaw`（不融合绝对
  x/y/yaw）；IMU 融合 `yaw`+`vyaw`（`ax/ay` 也不融合：互补滤波俯仰残差 ~1° 经 r_l 去重力
  会漏出 ~0.15 m/s² 假加速度，比本车最大线加速度还大）。轮速里程计的可信量是"走了多远、
  转得多快"，航向交给陀螺。
- **验证**：同一航点回归（`nav_waypoints --loop 2`，无头）修复前后对比：
  轮速 yaw 误差都涨到 1.9-3.1 rad（打滑是物理特性，消不掉），但 EKF 输出 yaw 误差从
  **1.5 rad 降到 ≤0.18 rad（典型 0.03）**，`map->odom` 修正量从 -1.36 rad 降到 -0.08 rad，
  前四段航点（客厅→门洞→厨房→回门洞）双向通过。
- **边界**：航点表 `table_ne`/`bedroom_door` 两段的失败**与此无关**——那是 0.25-0.35m 窄缝
  对 `inflation_radius 0.25` 的几何问题（规划器对 (5.8,3.2) 直接报
  `GridBased failed to generate a valid path`），修复前后同样失败，另案调参。
