# compute_command()

DWA 메인 진입 메서드. 입력 `(grid, resolution, center_x, center_y)`,
출력 `(v, w, min_clearance, fallback_used, obstacle_count)`.

1. [[get_front_obstacle_cells]] (Real: −x)
2. `nearest_obstacle_cells()` → `select_linear_speed()` (cruise/avoid/stop, Real은 stop>0)
3. w 후보 17개 → [[simulate_trajectory]] → [[score_trajectory_on_grid]]
4. 안전 후보 best 선택, 전부 reject면 [[fallback_rotation]]

호출자: [[go2_control]] (via [[go2_dwa_real]])
