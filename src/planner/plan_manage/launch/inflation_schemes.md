# 膨胀与局部地图参数说明（Inflation Schemes）

本文档说明三种对 `grid_map/obstacles_inflation` 与 `grid_map/local_map_margin` 的配置方案，以及如何配合 `optimization/dist0`，用于在不同风险/性能权衡下调整避障参数。

前提与计算规则

- `resolution`：栅格分辨率（m），本工程默认在 `advanced_param_exp.xml` 为 `0.15` m。
- `inf_step = ceil(obstacles_inflation / resolution)`。
- 实际生效膨胀半径 = `inf_step * resolution`。
- `local_map_margin` 的实际缓冲米数 = `local_map_margin * resolution`。
- 规划器的 `optimization/dist0` 应与“实际生效膨胀半径”保持一致或略大（+0.05–0.1 m），以避免规划器与栅格判定不一致。

无人机尺寸参考（来自用户输入）：
- 机高：0.40 m（不影响水平膨胀计算）
- 机长/机宽：0.55 m（中心到边缘 ≈ 0.275 m）
- 对角中心到角落半径 r ≈ 0.389 m（用于水平避障参考）

方案一：保守（推荐先行使用）
- 目标生效膨胀半径：0.60 m
- 设置：`grid_map/obstacles_inflation = 0.6`
- 对应离散步骤：`inf_step = ceil(0.6/0.15) = 4` → 实际生效 = `4 * 0.15 = 0.6 m`
- `local_map_margin` 建议 ≥ `inf_step + 2` → ≥ 6（当前值 10 足够）
- 同步设置：`optimization/dist0 = 0.6`（或 0.65 以留裕量）
- 适用场景：感知/TF 不稳定、环境狭窄或对碰撞容忍度低时。
- 权衡：最安全但通行率最低，可能在狭窄处出现无法通过的情况。

方案二：平衡（通行率与安全的折中）
- 目标生效膨胀半径：0.45 m
- 设置：`grid_map/obstacles_inflation = 0.45`
- 对应离散步骤：`inf_step = ceil(0.45/0.15) = 3` → 实际生效 = `3 * 0.15 = 0.45 m`
- `local_map_margin` 建议 ≥ 5
- 同步设置：`optimization/dist0 = 0.45`（或 +0.05）
- 适用场景：感知质量良好、对通过性有一定要求的常规场景。

方案三：精细（提高分辨率，精确匹配机体）
- 思路：降低 `resolution`（例如 0.10 或 0.05 m），再选择接近真实半径的 `obstacles_inflation`，能更准确匹配机体几何。
- 示例（resolution=0.10）：若目标膨胀 = 0.5 m，`inf_step = 5` → 实际生效 = 0.5 m。
- 代价：更高的计算与内存开销；需要更稳定的点云与 TF 对齐。
- 适用场景：受限空间但有高质量感知/算力的部署。

验证与调优流程（推荐）
1. 应用一个方案（先用保守方案）。
2. 在 RViz 同时显示：`/grid_map/occupancy_inflate`（膨胀）、机器人 mesh（`robot` topic）与规划轨迹。观察膨胀是否覆盖机体并留有合理余量。
3. 在仿真或低速试飞中验证是否仍发生碰撞或过度退避；若过度保守，可切换到平衡方案或降低 `resolution`（采取方案三）。
4. 若发现栅格与 planner 判定不一致，优先调整 `optimization/dist0` 以与栅格一致。

快速总结（默认文件修改）
- 当前已将 `advanced_param_exp.xml` 更新为保守方案：
  - `grid_map/obstacles_inflation = 0.6`
  - `optimization/dist0 = 0.6`
  - `grid_map/local_map_margin` 保持为 `10`（对应 1.5 m 的边界缓冲，通常足够）

如需我把平衡或精细方案也直接写入为备用 launch 文件或注释掉备份条目，我可以继续创建 `advanced_param_exp_balanced.xml` 或在本文件中保留注释备选项。