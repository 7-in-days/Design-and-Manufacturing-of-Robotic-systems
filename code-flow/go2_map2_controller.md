# go2_map2_controller

ROS2 노드 `Go2Control`, Map2용. `Isaac_sim=True`, 누적 **3 frame**.

- 구독: [[occupancy_map (topic)]] / 발행: [[cmd_vel (topic)]]
- planner: [[go2_dwa_sim_map2]] import
- 구조는 [[go2_map1_controller]]와 동일하나 frame 누적량과 fallback 로직이 다르다.

연관: [[compute_command]], [[fallback_rotation]], [[_Overview]]
