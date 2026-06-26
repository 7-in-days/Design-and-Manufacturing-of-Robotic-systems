#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard teleoperation tool for lookup-table calibration.

Use this tool to manually tune motor positions and print snippets that can be
copied into TARGET_TABLE or TRANSITION_TABLE.
"""

from __future__ import annotations

import rclpy
import time
from typing import Dict, Optional

try:
    from . import config
    from .controller import ContinuumController
except ImportError:
    import config
    from controller import ContinuumController


MIN_POSITION = 0
MAX_POSITION = 4095
DEFAULT_STEP = 10
DEFAULT_MOVE_TIME = 0.15


HELP_TEXT = """
Commands
  q / a : motor 0 +step / -step
  w / s : motor 1 +step / -step
  z / x : both motors +step / -step
  + / - : increase/decrease step size
  p     : print current motor feedback
  g     : go to typed motor positions
  h     : save current position as approach
  f     : save current position as final
  o     : print TARGET_TABLE snippet
  t     : print TRANSITION_TABLE snippet
  ?     : show this help
  exit  : quit
"""


def clamp_position(position: int) -> int:
    return max(MIN_POSITION, min(MAX_POSITION, int(position)))


def positions_from_feedback(controller: ContinuumController) -> Dict[int, int]:
    states = controller.get_realtime_position()
    return {
        motor_id: states[motor_id]["position"]
        for motor_id in controller.motor_ids
    }


def set_positions(
    controller: ContinuumController,
    positions: Dict[int, int],
    move_time: float = DEFAULT_MOVE_TIME,
) -> bool:
    targets = {
        motor_id: clamp_position(position)
        for motor_id, position in positions.items()
    }
    return controller.move_to_position_smooth(
        targets,
        move_time=move_time,
        rate_hz=30.0,
    )


def nudge_motor(
    controller: ContinuumController,
    positions: Dict[int, int],
    motor_id: int,
    delta: int,
) -> Dict[int, int]:
    targets = dict(positions)
    targets[motor_id] = clamp_position(targets[motor_id] + delta)
    if not set_positions(controller, targets):
        controller.get_logger().error("Failed to send nudge command")
    return targets


def nudge_all(
    controller: ContinuumController,
    positions: Dict[int, int],
    delta: int,
) -> Dict[int, int]:
    targets = dict(positions)
    for motor_id in controller.motor_ids:
        targets[motor_id] = clamp_position(targets[motor_id] + delta)
    if not set_positions(controller, targets):
        controller.get_logger().error("Failed to send nudge command")
    return targets


def print_position_dict(label: str, positions: Dict[int, int]) -> None:
    print(f"{label}: {{{', '.join(f'{mid}: {pos}' for mid, pos in positions.items())}}}")


def print_commanded_and_feedback(
    controller: ContinuumController,
    commanded: Dict[int, int],
) -> None:
    print_position_dict("Commanded", commanded)
    print_position_dict("Feedback", positions_from_feedback(controller))


def print_target_snippet(
    target_id: int,
    approach: Optional[Dict[int, int]],
    final: Optional[Dict[int, int]],
    move_time: float,
    settle_time: float,
) -> None:
    if approach is None or final is None:
        print("Save both approach(h) and final(f) before printing a snippet.")
        return

    print("\nTARGET_TABLE snippet")
    print(f"{target_id}: {{")
    print(f'    "approach": {approach},')
    print(f'    "final":    {final},')
    print(f'    "move_time": {move_time:.2f},')
    print(f'    "settle_time": {settle_time:.2f},')
    print("},\n")


def print_transition_snippet(
    prev_target: int,
    next_target: int,
    approach: Optional[Dict[int, int]],
    final: Optional[Dict[int, int]],
    move_time: float,
    settle_time: float,
) -> None:
    if approach is None or final is None:
        print("Save both approach(h) and final(f) before printing a snippet.")
        return

    print("\nTRANSITION_TABLE snippet")
    print(f"({prev_target}, {next_target}): {{")
    print(f'    "approach": {approach},')
    print(f'    "final":    {final},')
    print(f'    "move_time": {move_time:.2f},')
    print(f'    "settle_time": {settle_time:.2f},')
    print("},\n")


def read_int(prompt: str, default: Optional[int] = None) -> int:
    while True:
        suffix = f" [{default}]" if default is not None else ""
        text = input(f"{prompt}{suffix}: ").strip()
        if not text and default is not None:
            return default
        try:
            return int(text)
        except ValueError:
            print("Please enter an integer.")


def read_float(prompt: str, default: float) -> float:
    while True:
        text = input(f"{prompt} [{default}]: ").strip()
        if not text:
            return default
        try:
            return float(text)
        except ValueError:
            print("Please enter a number.")


def read_bool(prompt: str, default: bool) -> bool:
    default_text = "Y/n" if default else "y/N"
    while True:
        text = input(f"{prompt} [{default_text}]: ").strip().lower()
        if not text:
            return default
        if text in ("y", "yes"):
            return True
        if text in ("n", "no"):
            return False
        print("Please enter y or n.")


def read_initial_position_wizard(controller: ContinuumController) -> Optional[Dict[int, int]]:
    initial_position = getattr(
        config,
        "INITIAL_ENCODER_POSITION",
        getattr(config, "START_POSITION", None),
    )
    if initial_position is None:
        initial_position = positions_from_feedback(controller)

    print_position_dict("Configured initial encoder position", initial_position)
    if not read_bool("Edit initial encoder position for this teleop session?", False):
        return initial_position

    fallback_position = positions_from_feedback(controller)
    edited = {}
    for motor_id in controller.motor_ids:
        edited[motor_id] = clamp_position(
            read_int(
                f"Initial encoder for motor {motor_id}",
                initial_position.get(motor_id, fallback_position[motor_id]),
            )
        )

    print_position_dict("Session initial encoder position", edited)
    print("To make this permanent, copy this into config.py:")
    print(f"INITIAL_ENCODER_POSITION = {edited}")
    return edited


def main(args=None):
    controller = None

    try:
        rclpy.init(args=args)
        controller = ContinuumController()

        if not controller.check_connecting():
            controller.get_logger().error("Motor connection failed")
            return

        move_to_initial_default = getattr(config, "MOVE_TO_INITIAL_POSITION_ON_START", True)
        if read_bool("Move to initial encoder position before teleop?", move_to_initial_default):
            initial_position = read_initial_position_wizard(controller)
            initial_move_time = read_float(
                "Initial move_time",
                getattr(config, "INITIAL_MOVE_TIME", getattr(config, "START_MOVE_TIME", 1.0)),
            )
            if initial_position is not None and not set_positions(
                controller,
                initial_position,
                move_time=initial_move_time,
            ):
                controller.get_logger().error("Failed to reach initial encoder position")
                return

        print("\nLookup-table calibration teleop")
        mode = input("Calibrate target or transition? [target/transition]: ").strip().lower()
        if mode not in ("target", "transition"):
            mode = "target"

        target_id = read_int("Target id", 1)
        prev_target = None
        next_target = target_id
        if mode == "transition":
            prev_target = read_int("Previous target id", 1)
            next_target = read_int("Next target id", 2)

        move_time = read_float("Snippet move_time", 0.60)
        settle_time = read_float("Snippet settle_time", 0.25)

        step = DEFAULT_STEP
        approach = None
        final = None
        positions = positions_from_feedback(controller)

        print(HELP_TEXT)
        print_position_dict("Current", positions)

        while rclpy.ok():
            command = input(f"teleop step={step}> ").strip().lower()

            if command in ("exit", "quit"):
                break
            if command == "?":
                print(HELP_TEXT)
            elif command == "q":
                positions = nudge_motor(controller, positions, controller.motor_ids[0], step)
            elif command == "a":
                positions = nudge_motor(controller, positions, controller.motor_ids[0], -step)
            elif command == "w":
                positions = nudge_motor(controller, positions, controller.motor_ids[1], step)
            elif command == "s":
                positions = nudge_motor(controller, positions, controller.motor_ids[1], -step)
            elif command == "z":
                positions = nudge_all(controller, positions, step)
            elif command == "x":
                positions = nudge_all(controller, positions, -step)
            elif command in ("+", "="):
                step = min(500, step + 5)
                print(f"step = {step}")
            elif command in ("-", "_"):
                step = max(1, step - 5)
                print(f"step = {step}")
            elif command == "p":
                controller.print_realtime_feedback()
                positions = positions_from_feedback(controller)
            elif command == "g":
                next_positions = {}
                for motor_id in controller.motor_ids:
                    next_positions[motor_id] = read_int(
                        f"Motor {motor_id} position",
                        positions[motor_id],
                    )
                if set_positions(controller, next_positions):
                    positions = {
                        motor_id: clamp_position(position)
                        for motor_id, position in next_positions.items()
                    }
                else:
                    controller.get_logger().error("Failed to send typed positions")
            elif command == "h":
                approach = positions_from_feedback(controller)
                print_position_dict("Saved approach", approach)
            elif command == "f":
                final = positions_from_feedback(controller)
                print_position_dict("Saved final", final)
            elif command == "o":
                print_target_snippet(
                    target_id=target_id,
                    approach=approach,
                    final=final,
                    move_time=move_time,
                    settle_time=settle_time,
                )
            elif command == "t":
                if prev_target is None:
                    print("Transition mode was not selected at startup.")
                else:
                    print_transition_snippet(
                        prev_target=prev_target,
                        next_target=next_target,
                        approach=approach,
                        final=final,
                        move_time=move_time,
                        settle_time=settle_time,
                    )
            else:
                print("Unknown command. Type ? for help.")

            if command in ("q", "a", "w", "s", "z", "x", "g"):
                print_commanded_and_feedback(controller, positions)
            elif command not in ("?", "+", "=", "-", "_", "o", "t"):
                print_position_dict("Current", positions_from_feedback(controller))
                time.sleep(0.05)

    except KeyboardInterrupt:
        if controller:
            controller.get_logger().info("\nUser interrupt")
    except Exception as e:
        if controller:
            controller.get_logger().error(f"Error: {e}")
        else:
            print(f"Error: {e}")
    finally:
        if controller is not None:
            controller.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
