# score_trajectory_on_grid()

예측 궤적을 grid에 투영해 평가(Real은 `forward_xy_to_grid` −x 반전).
- 충돌 반경 내 장애물 → **hard reject**(`-1e18`)
- cost: `progress(+5) − heading(0.8) − lateral(0.4) − angular(0.6) − obstacle_count(0.8) − close_distance(5/clearance)`

입력: [[simulate_trajectory]] + [[get_front_obstacle_cells]]. 호출: [[compute_command]]
