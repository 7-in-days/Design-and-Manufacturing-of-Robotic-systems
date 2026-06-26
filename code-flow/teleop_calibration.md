# teleop_calibration() (mission 내 함수)

표적이 안 맞을 때 키보드(numpad)로 motor position 미세 조정 후 보정 position 반환.
각 입력마다 [[controller]] `_write_goal_position_with_recovery()` 호출.

정의: [[mission]] → 보정 delta는 [[save_calibration]]로 저장
도구 버전: [[teleop_calibration_tool]]
