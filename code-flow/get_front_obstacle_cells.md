# get_front_obstacle_cells()

`grid >= 50` cell 중 전방 ROI(전방 ~0.06–2.7 m, 좌우 ±1.5 m)만 추출.
**Real: `forward_cells = center_x − cols` (−x 전방)** — Sim과 부호 반대.

입력: [[occupancy_map (topic)]] · 호출: [[compute_command]]
