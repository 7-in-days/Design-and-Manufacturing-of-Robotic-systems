# Go2 Sim — Code Flow (Overview)

Obsidian Vault입니다. **Graph View(Ctrl/Cmd+G)** 를 켜면 아래 노트들의 `[[링크]]` 관계가
코드 흐름 그래프로 보입니다.

## 진입점 (ROS2 nodes)
- [[go2_map1_controller]] — Map1 노드 → [[go2_dwa_sim_map1]]
- [[go2_map2_controller]] — Map2 노드 → [[go2_dwa_sim_map2]]
- [[go2_sim_universal]] — planner 내장 통합 노드
- [[occupancy_map_rviz_debug]] — RViz 시각화 디버그 노드

## DWA 파이프라인
[[compute_command]] → [[get_front_obstacle_cells]] → [[simulate_trajectory]] →
[[score_trajectory_on_grid]] → [[fallback_rotation]]

## ROS2 토픽
- 입력: [[occupancy_map (topic)]]
- 출력: [[cmd_vel (topic)]]
