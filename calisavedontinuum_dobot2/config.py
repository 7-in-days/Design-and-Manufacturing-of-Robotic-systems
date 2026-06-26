#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission configuration for the continuum robot controller.

Edit these values directly instead of passing terminal arguments.
"""

# Scenario format examples: "1-2-3", "1 2 3", "1,2,3", "123"
SCENARIO_TEXT = "3-2-1-2-1-3-1-3-2-1-2-3-2-3-1-2-1-3-1-2-3-1-3-2-3-1-2-3-2-1"

MOTOR_NUM = [0, 1]

# User-defined initial encoder position for each motor.
#
# Edit these raw Dynamixel encoder values directly when you want the robot to
# move to a known initial pose at startup. XL430 position mode uses 0-4095;
# 2048 is the mechanical center for a normally mounted single-turn setup.
INITIAL_ENCODER_POSITION = {0: 2048, 1: 2048}
INITIAL_MOVE_TIME = 1.0

# Move to INITIAL_ENCODER_POSITION when main.py or teleop_calibration.py starts.
MOVE_TO_INITIAL_POSITION_ON_START = True

# Backward-compatible names used by older code paths.
START_POSITION = INITIAL_ENCODER_POSITION
START_MOVE_TIME = INITIAL_MOVE_TIME

# Mission timing parameters in seconds.
HOLD_TIME = 2
MISSION_TIME = 120.0
STABLE_TIMEOUT = 0.4

# When True, run the basic position-control test sequence instead of a mission.
TEST_MODE = False

# Initial realtime feedback sampling before mission/test execution.
INITIAL_FEEDBACK_COUNT = 3
INITIAL_FEEDBACK_INTERVAL = 0.5

# Dynamixel communication settings.
# XL430 and most X-series Dynamixel motors use Protocol 2.0.
DYNAMIXEL_PROTOCOL_VERSION = 2.0

# Number of retries after an automatic hardware-alert reboot.
HARDWARE_ALERT_RETRY_COUNT = 1
