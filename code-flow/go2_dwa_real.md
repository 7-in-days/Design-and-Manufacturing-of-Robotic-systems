# go2_dwa_real

`OccupancyDWAPlanner` (Real). 전방 좌표 **−x**: `forward = center_x − cols`,
`forward_xy_to_grid`도 부호 반전. 속도 cruise 1.2 / avoid 0.7 / **stop 0.5(완전 정지 X)**.

흐름: [[compute_command]] → [[get_front_obstacle_cells]] → [[simulate_trajectory]] →
[[score_trajectory_on_grid]] → [[fallback_rotation]]

사용처: [[go2_control]]
