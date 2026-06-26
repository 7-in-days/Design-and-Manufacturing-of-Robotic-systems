#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission execution utilities for continuum robot scenario control.
"""

from typing import Dict, Optional

try:
    from .scenario import parse_scenario
except ImportError:
    from scenario import parse_scenario


def get_motion(prev_target: Optional[int], next_target: int) -> Dict:
    """
    Get motion parameters for transition.
    
    Args:
        prev_target: Previous target (None for first)
        next_target: Next target
    
    Returns:
        Motion dict with approach, final, move_time, settle_time
    
    Raises:
        ValueError: If next_target is not in TARGET_TABLE
    """
    try:
        from .calibration import TARGET_TABLE, TRANSITION_TABLE
    except ImportError:
        from calibration import TARGET_TABLE, TRANSITION_TABLE

    if next_target not in TARGET_TABLE:
        raise ValueError(f"Target {next_target} not found in TARGET_TABLE")
    
    if prev_target is None:
        return TARGET_TABLE[next_target]
    
    key = (prev_target, next_target)
    if key in TRANSITION_TABLE:
        return TRANSITION_TABLE[key]
    else:
        return TARGET_TABLE[next_target]


def run_mission_sequence(
    controller: "ContinuumController",
    scenario_text: str,
    hold_time: float = 2.05,
    mission_time: float = 60.0,
    stable_timeout: float = 0.4
) -> int:
    """
    Run mission sequence with timing constraints.
    
    Open-loop mission execution without camera feedback.
    Relies on pre-calibrated lookup table (TARGET_TABLE, TRANSITION_TABLE).
    
    Args:
        controller: ContinuumController instance
        scenario_text: Scenario string (e.g., "1-2-3" or "1 2 3")
        hold_time: Hold time per target (seconds, default 2.05 for 2+ seconds stability)
        mission_time: Total mission time limit (seconds, default 60.0)
        stable_timeout: Stabilization timeout per target (seconds, default 0.4)
    
    Returns:
        Number of successfully completed targets
    """
    try:
        sequence = parse_scenario(scenario_text)
    except ValueError as e:
        controller.get_logger().error(f"Scenario parsing failed: {e}")
        return 0
    
    # Check sequence length (warning only, continue execution for testing)
    if len(sequence) != 30:
        controller.get_logger().warning(
            f"Sequence length is {len(sequence)}, expected 30 targets. "
            "Continuing for testing purposes.")
    
    import time
    start_time = time.time()
    prev_target = None
    completed = 0
    
    controller.get_logger().info(f"Mission start: {len(sequence)} targets, {mission_time}s limit")
    
    for i, target in enumerate(sequence):
        elapsed = time.time() - start_time
        remaining = mission_time - elapsed
        
        controller.get_logger().info(f"[{i+1}/{len(sequence)}] Target {target}, Time left: {remaining:.1f}s")
        
        try:
            # Get motion parameters (approach and final positions)
            motion = get_motion(prev_target, target)
            
            # Calculate estimated required time for this target
            estimated_required = motion["move_time"] + stable_timeout + hold_time
            
            # Check if enough time remains for this target
            if remaining <= estimated_required:
                controller.get_logger().warning(
                    f"Insufficient time for target {target}: "
                    f"remaining={remaining:.1f}s, required={estimated_required:.1f}s")
                break
            
            # Approach move (60% of move_time for smooth approach)
            approach_time = motion["move_time"] * 0.6
            controller.get_logger().info(f"  Approach move ({approach_time:.2f}s)...")
            if not controller.move_to_position_smooth(
                motion["approach"], 
                move_time=approach_time,
                rate_hz=30.0
            ):
                controller.get_logger().error(f"  Approach failed for target {target}")
                break
            
            # Final move (40% of move_time for precise final positioning)
            final_time = motion["move_time"] * 0.4
            controller.get_logger().info(f"  Final move ({final_time:.2f}s)...")
            if not controller.move_to_position_smooth(
                motion["final"],
                move_time=final_time,
                rate_hz=30.0
            ):
                controller.get_logger().error(f"  Final move failed for target {target}")
                break
            
            # Wait until stable at final position
            controller.get_logger().info(f"  Stabilizing ({motion['settle_time']:.2f}s)...")
            if not controller.wait_until_stable(
                motion["final"],
                position_tolerance=10,
                velocity_threshold=5,
                stable_duration=motion["settle_time"],
                timeout=stable_timeout
            ):
                controller.get_logger().warning(f"  Stabilization timeout for target {target}, continuing")
            
            # Hold at final position
            hold_start = time.time()
            controller.get_logger().info(f"  Holding ({hold_time:.2f}s)...")
            while time.time() - hold_start < hold_time:
                controller.get_realtime_position()
                time.sleep(0.1)
            
            prev_target = target
            completed += 1
            controller.get_logger().info(f"  Target {target} completed")
            
        except Exception as e:
            controller.get_logger().error(f"Error at target {target}: {e}")
            break
    
    controller.get_logger().info(f"Mission complete: {completed}/{len(sequence)} targets completed")
    return completed
