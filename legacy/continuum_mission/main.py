#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main entry point for continuum robot mission controller.
"""

import rclpy
import time

try:
    from . import config
    from .controller import ContinuumController
    from .mission import run_mission_sequence
except ImportError:
    import config
    from controller import ContinuumController
    from mission import run_mission_sequence


def main(args=None):
    """
    Main entry point for continuum robot mission control.
    
    Supports mission-based scenario execution with timing constraints.
    Uses pre-calibrated lookup tables for open-loop control without camera feedback.
    
    Scenario format examples:
      - "1-2-3-1-3-2"
      - "1 2 3 1 3 2"
      - "1,2,3,1,3,2"
      - "123132"
    
    Optional test mode executes basic position control sequence.
    """
    controller = None
    
    try:
        rclpy.init(args=args)
        controller = ContinuumController()
        
        # Step 1: Check motor connections
        if not controller.check_connecting():
            controller.get_logger().error("Motor connection failed")
            return
        
        # Step 2: Initial feedback check
        controller.get_logger().info("\nInitial realtime feedback...")
        for _ in range(config.INITIAL_FEEDBACK_COUNT):
            controller.print_realtime_feedback()
            time.sleep(config.INITIAL_FEEDBACK_INTERVAL)

        # Step 3: Move to configured initial encoder position before execution.
        move_to_initial = getattr(config, "MOVE_TO_INITIAL_POSITION_ON_START", True)
        initial_position = getattr(
            config,
            "INITIAL_ENCODER_POSITION",
            getattr(config, "START_POSITION", None),
        )
        if move_to_initial and initial_position is not None:
            initial_move_time = getattr(
                config,
                "INITIAL_MOVE_TIME",
                getattr(config, "START_MOVE_TIME", 1.0),
            )
            controller.get_logger().info(
                f"\nMoving to initial encoder position: {initial_position} "
                f"({initial_move_time:.2f}s)"
            )
            if not controller.move_to_position_smooth(
                initial_position,
                move_time=initial_move_time,
                rate_hz=30.0,
            ):
                controller.get_logger().error("Failed to reach initial encoder position")
                return
        
        # Step 4: Execute configured test mode or lookup-table mission scenario.
        if config.TEST_MODE:
            controller.get_logger().info("\n" + "=" * 70)
            controller.get_logger().info("Running TEST sequence (basic position control)")
            controller.get_logger().info("=" * 70)
            
            test_sequence = [
                {0: 1000, 1: 1500},  # Position 1
                {0: 2500, 1: 2500},  # Position 2 (middle)
                {0: 3500, 1: 3000},  # Position 3
                {0: 2048, 1: 2048},  # Home position
            ]
            
            if controller.move_sequence(
                test_sequence,
                hold_time=config.HOLD_TIME,
                timeout_per_move=10.0
            ):
                controller.get_logger().info("Test sequence completed successfully")
            return
        
        scenario_text = config.SCENARIO_TEXT.strip()
        if not scenario_text:
            controller.get_logger().error("Empty scenario")
            return
        controller.get_logger().info(f"Scenario from config.py: {scenario_text}")
        
        # Step 5: Execute mission sequence
        controller.get_logger().info("\n" + "=" * 70)
        controller.get_logger().info("MISSION EXECUTION")
        controller.get_logger().info("=" * 70)
        
        completed = run_mission_sequence(
            controller,
            scenario_text=scenario_text,
            hold_time=config.HOLD_TIME,
            mission_time=config.MISSION_TIME,
            stable_timeout=config.STABLE_TIMEOUT
        )
        
        controller.get_logger().info(f"\nMission Result: {completed} targets completed")
        
    except KeyboardInterrupt:
        if controller:
            controller.get_logger().info("\nUser interrupt")
    except Exception as e:
        if controller:
            controller.get_logger().error(f"Error: {e}")
    finally:
        if controller is not None:
            controller.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
