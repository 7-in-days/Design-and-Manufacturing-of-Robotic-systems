# score_trajectory_on_grid()

예측 궤적을 grid에 투영해 평가.
- 충돌 반경 안 장애물 → **hard reject** (`-1e18`)
- cost: `progress(+5) − heading(0.8) − lateral(0.4) − angular(0.6) − obstacle_count(0.8) − close_distance(5/clearance)`

입력: [[simulate_trajectory]] 결과 + [[get_front_obstacle_cells]] 장애물.
호출: [[compute_command]]
