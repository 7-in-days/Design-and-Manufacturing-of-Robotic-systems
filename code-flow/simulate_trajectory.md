# simulate_trajectory(v, w)

고정 `(v, w)`로 3초(30 step, dt=0.1) 미래 궤적 `(x, y, yaw)` 예측.
각 후보마다 호출되며 결과는 [[score_trajectory_on_grid]]로 전달된다.

호출: [[compute_command]]
