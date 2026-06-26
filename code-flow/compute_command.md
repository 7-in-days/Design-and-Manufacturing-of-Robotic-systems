# compute_command()

DWA 메인 진입 메서드. 입력: `(grid, resolution, center_x, center_y)`,
출력: `(v, w, min_clearance, fallback_used, obstacle_count)`.

순서:
1. [[get_front_obstacle_cells]] — ROI 전방 장애물 추출
2. `nearest_obstacle_cells()` → `select_linear_speed()` (cruise/avoid/stop)
3. v≈0 & 장애물 → 즉시 [[fallback_rotation]]
4. w 후보 17개 → [[simulate_trajectory]] → [[score_trajectory_on_grid]]
5. 안전 후보 중 best 선택, 전부 reject면 [[fallback_rotation]]

호출자: [[go2_map1_controller]], [[go2_map2_controller]], [[go2_sim_universal]]
