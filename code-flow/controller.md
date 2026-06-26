# controller.py — ContinuumController

Dynamixel **SDK PacketHandler** 저수준 wrapper (메인 사용).
주요 메서드: `check_connecting`, `get_realtime_position`, [[move_to_position_smooth]],
[[wait_until_stable]], `move_sequence`, `recover_from_hardware_alerts`, `reboot_motor`, `shutdown`.

imports: [[config]], [[trajectory]], [[dynamixel_sdk]]
호출자: [[main]], [[mission]], [[teleop_calibration_tool]]
대안: [[continumm_controller]]
