# RViz @ WSL2 渲染类

## RViz 3D 视口铺满黄色（黄/青条纹）垃圾，小车一动黄色加深直至全黄
两个独立触发源叠加，均与 SLAM 算法无关（换 gmapping 不解决）。

### 触发源 1：llvmpipe 多线程光栅化 tile bug
- **现象**：视口满屏黄/青相间竖条纹，均值呈中性灰；Grid/Map/RobotModel/TF/LaserScan 全部显示时出现。
- **原因**：WSL2 上 `LIBGL_ALWAYS_SOFTWARE=1` 使 rviz2 走 llvmpipe，其多线程光栅器（Mesa 23.2.1）tile 拼装错位。
- **解决**：`LP_NUM_THREADS=1`（launch 里已对 rviz2 节点用 `additional_env` 设置）。

### 触发源 2：Odometry 显示项（WSL 重启后出现，2026-10-05）
- **现象**：即使 `LP_NUM_THREADS=1`，视口仍满屏条纹；**逐显示项二分后定位：只开 Odometry 即复现，其余五个显示项单独开全干净**；Shape 改 Axes 无效（与箭头网格无关）。
- **原因**：rviz2 Odometry 显示项的渲染路径在 Mesa 软件渲染下触发视口级坏渲染；WSL 重启（宿主侧 WSLg 组件可能更新）后从"被线程修复掩盖"变为直接复现。glxgears 干净、rviz 的 Qt 面板干净，说明问题限定在 Ogre 显示项内部，不是窗口系统层。
- **解决**：slam.rviz 中 Odometry 显示项 `Enabled: false`（已改，里程计轨迹非建图必需）。其他配置文件要用 Odometry 前先按下面方法验证。
- **鉴别/定位方法**（通用，值得复用）：
  1. `xwd -id <窗口id>` 抓屏 → 解析统计视口区颜色：健康时暗背景(RGB 48,48,48)占绝对多数，出问题时黄/青像素各约 20-30%。
  2. 嫌疑收敛后做**最小配置二分**：手写仅含单个显示项的 .rviz 逐个试，垃圾随哪个显示项出现即为元凶。
  3. 对照实验要固定变量：gui 与否、窗口大小、驱动（llvmpipe/softpipe/d3d12）逐一排除；d3d12 路径 rviz 会直接段错误（勿用）。
- **验证**：禁用 Odometry 后抓屏统计 黄0% 青0%、暗背景 74-98%、地图自由区正常显示。
- **残留告警**：Map 显示的 `indexed_8bit_image ... GLSL link` 报错（"active samplers..."）在 Mesa 上始终存在，实测不影响渲染，可忽略。
