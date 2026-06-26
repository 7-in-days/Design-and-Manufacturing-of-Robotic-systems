# main.py (entry point)

1. `rclpy.init()` → [[controller]] 생성
2. `check_connecting()` (모터 ping/torque/모드 초기화)
3. 초기 자세로 [[move_to_position_smooth]]
4. [[scenario]] `parse_scenario(config.SCENARIO_TEXT)`
5. [[mission]] `run_mission_sequence(...)`
6. `shutdown()`

uses: [[config]] · 연관: [[_Overview]]
