# Go2 Real — Code Flow (Overview)

Obsidian Vault입니다. **Graph View(Ctrl/Cmd+G)** 로 `[[링크]]` 관계를 그래프로 봅니다.

## 진입점 (ROS2 node)
- [[go2_control]] — Real 노드 (`Isaac_sim=False`) → [[go2_dwa_real]]
- [[occupancy_map_rviz_debug]] — RViz 시각화 디버그 노드
- [[go2_dwa_sim_map1]] / [[go2_dwa_sim_map2]] — Sim용 planner (Real 실행 시 미사용)

## DWA 파이프라인
[[compute_command]] → [[get_front_obstacle_cells]] → [[simulate_trajectory]] →
[[score_trajectory_on_grid]] → [[fallback_rotation]]

## ROS2 토픽
- 입력: [[occupancy_map (topic)]] · 출력: [[cmd_vel (topic)]]
