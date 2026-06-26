#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission execution utilities for continuum robot scenario control.
"""

import os
import sys
import termios
import tty
import time
from typing import Dict, Optional

try:
    from .scenario import parse_scenario
    from .calibration import TARGET_TABLE, TRANSITION_TABLE, TELEOP_STEP_SCALE
except ImportError:
    from scenario import parse_scenario
    from calibration import TARGET_TABLE, TRANSITION_TABLE, TELEOP_STEP_SCALE

def save_calibration(controller, target_table):
    calib_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.py')
    try:
        with open(calib_file_path, 'w', encoding='utf-8') as f:
            f.write('#!/usr/bin/env python3\n')
            f.write('# -*- coding: utf-8 -*-\n')
            f.write('"""\nCalibration lookup tables for continuum robot targets.\n"""\n\n')
            f.write(f'TELEOP_STEP_SCALE = {TELEOP_STEP_SCALE}\n\n')
            f.write('TARGET_TABLE = {\n')
            for k, v in target_table.items():
                f.write(f'    {k}: {{\n')
                f.write(f'        "approach": {{0: {v["approach"][0]}, 1: {v["approach"][1]}}},\n')
                f.write(f'        "final":    {{0: {v["final"][0]}, 1: {v["final"][1]}}},\n')
                f.write(f'        "move_time": {v["move_time"]},\n')
                f.write(f'        "settle_time": {v["settle_time"]},\n')
                f.write('    },\n')
            f.write('}\n\n')
            f.write('TRANSITION_TABLE = {}\n')
        controller.get_logger().info("calibration.py 파일에 자동 저장 완료.")
    except Exception as e:
        controller.get_logger().error(f"파일 저장 실패: {e}")

def get_motion(prev_target: Optional[int], next_target: int) -> Dict:
    """
    Get motion parameters for transition.
    """
    if next_target not in TARGET_TABLE:
        raise ValueError(f"Target {next_target} not found in TARGET_TABLE")
    
    if prev_target is None:
        return TARGET_TABLE[next_target]
    
    key = (prev_target, next_target)
    if key in TRANSITION_TABLE:
        return TRANSITION_TABLE[key]
    else:
        return TARGET_TABLE[next_target]

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def teleop_calibration(controller, target_num: int, current_pos: Dict[int, int]) -> Dict[int, int]:
    controller.get_logger().info("\n----------------------------------------")
    controller.get_logger().info(f"Target {target_num} 미세 조정 모드 진입")
    controller.get_logger().info("텐키(NumPad)를 사용하여 위치를 조정. 완료 시 Enter 입력.")
    
    if target_num == 1:
        controller.get_logger().info("방위 매핑 [Target 1]:")
        controller.get_logger().info("8: 위(0+) / 2: 아래(0-) / 4: 왼(1-) / 6: 오(1+)")
        controller.get_logger().info("7: 왼위 / 9: 오위 / 1: 왼아래 / 3: 오아래")
    elif target_num == 2:
        controller.get_logger().info("방위 매핑 [Target 2]:")
        controller.get_logger().info("7: 왼위(0+) / 9: 오위(1+) / 3: 오아래(0-) / 1: 왼아래(1-)")
        controller.get_logger().info("8: 위 / 2: 아래 / 4: 왼 / 6: 오 (조합 적용됨)")
    elif target_num == 3:
        controller.get_logger().info("방위 매핑 [Target 3]:")
        controller.get_logger().info("8: 위(1+) / 2: 아래(1-) / 4: 왼(0+) / 6: 오(0-)")
        controller.get_logger().info("7: 왼위 / 9: 오위 / 1: 왼아래 / 3: 오아래")
        
    pos = current_pos.copy()
    scale = TELEOP_STEP_SCALE

    while True:
        key = get_key()
        if key in ('\r', '\n'):
            controller.get_logger().info("\n미세 조정 완료.")
            break
        
        d0 = 0
        d1 = 0
        
        if target_num == 1:
            if key == '8': d0 = 1
            elif key == '2': d0 = -1
            elif key == '4': d1 = -1
            elif key == '6': d1 = 1
            elif key == '7': d0 = 1; d1 = -1
            elif key == '9': d0 = 1; d1 = 1
            elif key == '1': d0 = -1; d1 = -1
            elif key == '3': d0 = -1; d1 = 1
        elif target_num == 2:
            if key == '7': d0 = 1
            elif key == '9': d1 = 1
            elif key == '3': d0 = -1
            elif key == '1': d1 = -1
            elif key == '8': d0 = 1; d1 = 1
            elif key == '2': d0 = -1; d1 = -1
            elif key == '4': d0 = 1; d1 = -1
            elif key == '6': d0 = -1; d1 = 1
        elif target_num == 3:
            if key == '8': d1 = 1
            elif key == '2': d1 = -1
            elif key == '4': d0 = 1
            elif key == '6': d0 = -1
            elif key == '7': d1 = 1; d0 = 1
            elif key == '9': d1 = 1; d0 = -1
            elif key == '1': d1 = -1; d0 = 1
            elif key == '3': d1 = -1; d0 = -1
            
        if d0 != 0 or d1 != 0:
            pos[0] += d0 * scale
            pos[1] += d1 * scale
            controller._write_goal_position_with_recovery(0, int(pos[0]), "teleop write")
            controller._write_goal_position_with_recovery(1, int(pos[1]), "teleop write")
            sys.stdout.write(f"\r현재 위치 -> M0: {pos[0]}, M1: {pos[1]}")
            sys.stdout.flush()
            
    return pos

def run_mission_sequence(
    controller: "ContinuumController",
    scenario_text: str,
    hold_time: float = 2.05,
    mission_time: float = 60.0,
    stable_timeout: float = 0.4,
    first_start_delay: float = 1.2
) -> int:
    try:
        sequence = parse_scenario(scenario_text)
    except ValueError as e:
        controller.get_logger().error(f"Scenario parsing failed: {e}")
        return 0
    
    if len(sequence) != 30:
        controller.get_logger().warning(
            f"Sequence length is {len(sequence)}, expected 30 targets. "
            "Continuing for testing purposes.")
    
    start_time = time.time()
    prev_target = None
    completed = 0
    
    controller.get_logger().info(f"Mission start: {len(sequence)} targets, {mission_time}s limit")
    
    for i, target in enumerate(sequence):
        if i == 0 and first_start_delay > 0:
            controller.get_logger().info(f"First start delay ({first_start_delay:.2f}s)...")
            time.sleep(first_start_delay)

        elapsed = time.time() - start_time
        remaining = mission_time - elapsed
        
        controller.get_logger().info(f"[{i+1}/{len(sequence)}] Target {target}, Time left: {remaining:.1f}s")
        
        try:
            motion = get_motion(prev_target, target)
            estimated_required = motion["move_time"] + stable_timeout
            
            if remaining <= estimated_required:
                controller.get_logger().warning(
                    f"Insufficient time for target {target}: "
                    f"remaining={remaining:.1f}s, required={estimated_required:.1f}s")
                break
            
            approach_time = motion["move_time"] * 0.6
            controller.get_logger().info(f"  Approach move ({approach_time:.2f}s)...")
            if not controller.move_to_position_smooth(
                motion["approach"], 
                move_time=approach_time,
                rate_hz=30.0
            ):
                controller.get_logger().error(f"  Approach failed for target {target}")
                break
            
            final_time = motion["move_time"] * 0.4
            controller.get_logger().info(f"  Final move ({final_time:.2f}s)...")
            if not controller.move_to_position_smooth(
                motion["final"],
                move_time=final_time,
                rate_hz=30.0
            ):
                controller.get_logger().error(f"  Final move failed for target {target}")
                break
            
            controller.get_logger().info(f"  Stabilizing ({motion['settle_time']:.2f}s)...")
            if not controller.wait_until_stable(
                motion["final"],
                position_tolerance=10,
                velocity_threshold=5,
                stable_duration=motion["settle_time"],
                timeout=stable_timeout
            ):
                controller.get_logger().warning(f"  Stabilization timeout for target {target}, continuing")
            
            calibrated_pos = teleop_calibration(controller, target, motion["final"].copy())
            
            delta_0 = calibrated_pos[0] - motion["final"][0]
            delta_1 = calibrated_pos[1] - motion["final"][1]
            
            if delta_0 != 0 or delta_1 != 0:
                TARGET_TABLE[target]["final"][0] += delta_0
                TARGET_TABLE[target]["final"][1] += delta_1
                TARGET_TABLE[target]["approach"][0] += delta_0
                TARGET_TABLE[target]["approach"][1] += delta_1
                controller.get_logger().info(
                    f"  Target {target} 테이블 메모리 갱신 완료 (M0: {delta_0:+}, M1: {delta_1:+})"
                )
                save_calibration(controller, TARGET_TABLE)
            
            prev_target = target
            completed += 1
            controller.get_logger().info(f"  Target {target} completed")
            
        except Exception as e:
            controller.get_logger().error(f"Error at target {target}: {e}")
            break
            
    controller.get_logger().info(f"Mission complete: {completed}/{len(sequence)} targets completed")
    return completed