# Gazebo 模型解析类

## `model://` 本地解析不到 → gzserver 连在线模型库卡死，表现为"Gazebo 无响应"

- **现象**：`ros2 launch car_bringup gazebo.launch.py` 后 Gazebo 窗口长期停在 "Preparing your world..."、拖不动（Windows/WSLg 会标"未响应"）；30 秒后 launch 日志报
  `[spawn_entity]: Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?` 并退出，车不出现。
  **注意这条报错极具误导性**——它指向插件的锅，实际跟插件无关。
- **原因**：`home_world.sdf` 里 `<include><uri>model://ground_plane</uri></include>` 和 `model://sun` 在本地搜索路径里找不到时，gzserver 会回落到**在线模型库** `http://models.gazebosim.org/`。该请求在本机网络下极慢或直接挂住（实测卡住时 `ss` 可见 40+ 条外连、零进展），于是：

  ```
  卡在 [Wrn] [ModelDatabase.cc:340] Getting models from[...]
    → 世界永远加载不完 → /spawn_entity 服务始终不注册
    → spawn_entity.py 默认 30s 超时退出 → gzclient 停在 "Preparing your world..."
  ```

  **为什么本地会找不到**（这是最反直觉的一环）：`gazebo_ros` 的 `gzserver.launch.py` 把 `GAZEBO_MODEL_PATH` / `GAZEBO_PLUGIN_PATH` / `GAZEBO_RESOURCE_PATH` **整个覆盖**成"各包 `<export><gazebo_ros/>` 导出的路径拼接"，**不是追加**。Gazebo 自带的 `/usr/share/gazebo-11/models`（`ground_plane`/`sun` 的唯一系统来源）因此不在搜索范围内，这两个模型只能从 `~/.gazebo/models/` 或某个包导出的路径里找到。
- **鉴别**（一眼定性，别去查插件/渲染/TF）：失败时
  ```bash
  tail ~/.gazebo/server-11345/default.log
  ```
  末行若是 `[Wrn] [ModelDatabase.cc:340] Getting models from[http://models.gazebosim.org/]` 就是本条；正常时该日志会直接出现 `[Msg] Loading world file [...]`，不会有 ModelDatabase 行。
  再顺手确认：`ls ~/.gazebo/models/ | grep -E 'ground_plane|sun'` 是否为空。
- **解决**：让世界用到的每个 `model://` 都能**离线**解析。本项目已把 `ground_plane` 与 `sun` 随包放在 `src/car_bringup/models/`，由 `src/car_bringup/package.xml` 导出：
  ```xml
  <export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
  </export>
  ```
  改 `package.xml` 后必须 `colcon build --symlink-install --packages-select car_bringup` 才生效。等价老办法是把模型拷进 `~/.gazebo/models/`（Gazebo 内置默认搜索路径），但那是机器级手工操作、不进 git，换机器就复发。
- **兜底**（可选）：`export GAZEBO_MODEL_DATABASE_URI=""` 可禁掉在线库——缺失的模型会快速报错并跳过（`Unable to find uri[model://xxx]`，世界照常加载），不再挂死。实测外连 0 条。适合排查期先"止血"。
- **验证**：
  ```bash
  # 把可能救场的机器级副本挪走，考察随包副本能否独立支撑（模拟全新机器）
  mv ~/.gazebo/models/{ground_plane,sun} /tmp/
  ros2 launch car_bringup gazebo.launch.py      # 应正常 spawn 出 xuegecar
  tail ~/.gazebo/server-11345/default.log       # 应无 ModelDatabase 行，有 Loading world file
  mv /tmp/{ground_plane,sun} ~/.gazebo/models/  # 测完放回
  ```
  实测通过：随包副本单独生效，且 gzserver 未往源码树写入任何 SDF 版本转换副本。
- **附带**：`~/.gazebo/models/` 下**每个子目录都必须有 `model.config`**，否则 gzclient 启动时报
  `[Err] [InsertModelWidget.cc:403] Missing model.config for model [...]`。把整个仿真工程（含 `models/ worlds/ meshes/`）直接丢进 `~/.gazebo/models/` 就会踩这个坑——`models/` 下应当只放单个模型目录。
- **防复发**：`spawn_entity.py` 的 `/spawn_entity` 等待超时默认只有 **30s**（`-timeout` 参数），`gazebo.launch.py` 已显式放宽到 120s。这只影响"等多久才判失败"，但真正的问题始终是本节这条，超时只是暴露它的方式。
