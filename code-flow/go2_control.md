# go2_control

Real용 ROS2 노드 `Go2Control` (name=`go2_control`), `Isaac_sim=False`.

- 구독: [[occupancy_map (topic)]] → `map_callback()` (**raw frame**, 누적 없음 → `latest_grid = grid`)
- 발행: [[cmd_vel (topic)]] ← `publish_cmd()`
- 10 Hz `control_loop()`에서 [[compute_command]] 호출
- planner: [[go2_dwa_real]] import

연관: [[_Overview]]
