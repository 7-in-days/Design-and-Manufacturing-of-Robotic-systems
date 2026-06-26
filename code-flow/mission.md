# mission.py

`run_mission_sequence()` — 미션 메인 루프. 각 target마다:
[[get_motion]] → approach [[move_to_position_smooth]] → final [[move_to_position_smooth]] →
[[wait_until_stable]] → [[teleop_calibration]] → (delta≠0) [[save_calibration]]

imports: [[scenario]], [[calibration]] · 호출: [[controller]]
호출자: [[main]]
