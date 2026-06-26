# get_front_obstacle_cells()

`grid >= 50`인 cell 중 전방 ROI(전방 ~0.06–2.7 m, 좌우 ±1.5 m, FOV ±90°)만 추출.
Sim: `forward_cells = cols − center_x` (**+x 전방**).

입력 제공: [[occupancy_map (topic)]] · 호출: [[compute_command]]
