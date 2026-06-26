# go2_dwa_sim_map1

`OccupancyDWAPlanner` (Map1). Sim 좌표: `forward = cols − center_x` (**+x 전방**).

핵심 메서드 흐름: [[compute_command]] → [[get_front_obstacle_cells]] →
[[simulate_trajectory]] → [[score_trajectory_on_grid]] → [[fallback_rotation]]

fallback 방향 결정 시 좌측 bias margin(40 cell) 존재.

사용처: [[go2_map1_controller]]
