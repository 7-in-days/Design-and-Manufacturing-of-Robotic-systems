# occupancy_map_rviz_debug

RViz 디버그 노드. 동일한 planner를 재사용해 모든 w 후보 경로/ROI/충돌 여부를 계산하고
`visualization_msgs/MarkerArray`(`/occupancy_map_debug/markers`)로 발행한다.

- 구독: [[occupancy_map (topic)]]
- 재사용: [[get_front_obstacle_cells]], [[simulate_trajectory]], [[score_trajectory_on_grid]]
연관: [[go2_dwa_real]], [[_Overview]]
