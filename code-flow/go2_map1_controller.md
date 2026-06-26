# go2_map1_controller

ROS2 노드 `Go2Control` (name=`go2_control`), Map1용. `Isaac_sim=True`, 누적 **10 frame**.

- 구독: [[occupancy_map (topic)]] → `map_callback()`
- 발행: [[cmd_vel (topic)]] ← `publish_cmd()`
- 10 Hz `control_loop()` 타이머에서 [[compute_command]] 호출
- planner: [[go2_dwa_sim_map1]] 의 `OccupancyDWAPlanner` import

`map_callback()`에서 최근 10 frame의 `grid >= 50` 마스크를 `np.logical_or`로 누적해
[[occupancy_map (topic)]]의 깜빡임을 억제한다.

연관: [[go2_map2_controller]], [[_Overview]]
