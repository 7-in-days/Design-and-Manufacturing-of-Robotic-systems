# Continuum Robot — Code Flow (Overview)

Obsidian Vault입니다. **Graph View(Ctrl/Cmd+G)** 로 `[[링크]]` 관계를 그래프로 봅니다.

## 진입점
- [[main]] → [[controller]] + [[mission]]

## 미션 파이프라인
[[main]] → [[scenario]] → [[mission]] → [[get_motion]] → [[move_to_position_smooth]] →
[[wait_until_stable]] → [[teleop_calibration]] → [[save_calibration]]

## 데이터 / 유틸
- lookup table: [[calibration]]
- 설정: [[config]]
- 보간: [[trajectory]] (minimum_jerk_scalar)
- 저수준: [[dynamixel_sdk]]
- 대안 구현(미사용): [[continumm_controller]]
- 도구: [[teleop_calibration_tool]], [[reboot_motor]], [[check_sdk]]
