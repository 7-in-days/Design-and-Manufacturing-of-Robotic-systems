# /occupancy_map (topic)

`nav_msgs/OccupancyGrid`, 400×400 cell, 해상도 0.03 m (~12×12 m).
값: 0/-1 = 빈공간/미확인, 100 = 장애물. 코드는 `>= 50`을 장애물로 판단.

구독: [[go2_map1_controller]], [[go2_map2_controller]], [[go2_sim_universal]], [[occupancy_map_rviz_debug]]
