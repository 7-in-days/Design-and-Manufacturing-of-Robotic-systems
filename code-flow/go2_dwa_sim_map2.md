# go2_dwa_sim_map2

`OccupancyDWAPlanner` (Map2). 흐름은 [[go2_dwa_sim_map1]]과 동일하나:
- fallback 방향 bias margin 없음(대칭)
- `update_fallback_clear_lock()` — 연속 clear frame 5회 후 lock 해제

흐름: [[compute_command]] → [[get_front_obstacle_cells]] →
[[simulate_trajectory]] → [[score_trajectory_on_grid]] → [[fallback_rotation]]

사용처: [[go2_map2_controller]]
