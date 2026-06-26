# move_to_position_smooth()

현재→목표 position을 여러 step으로 나눠 매 step `r=step/steps`에 대해
[[trajectory]] `minimum_jerk_scalar(r)`로 보간, `_write_goal_position_with_recovery()`로
[[dynamixel_sdk]] `GOAL_POSITION`에 기록(하드웨어 alert 시 reboot 복구).

정의: [[controller]] · 호출: [[main]], [[mission]], [[teleop_calibration_tool]]
