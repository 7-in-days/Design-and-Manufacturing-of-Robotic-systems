# fallback_rotation (get_locked_fallback_turn)

모든 후보가 reject되거나 v≈0일 때 제자리 회전으로 장애물이 적은 방향 탐색.
좌우 장애물 개수로 회전 방향 결정, 진동 억제를 위해 방향 lock 유지.

- Map1: 좌측 bias margin 40 cell ([[go2_dwa_sim_map1]])
- Map2: 대칭 + `update_fallback_clear_lock()` ([[go2_dwa_sim_map2]])

호출: [[compute_command]]
