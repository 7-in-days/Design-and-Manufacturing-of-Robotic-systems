#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration lookup tables for continuum robot targets.
"""

TELEOP_STEP_SCALE = 5

TARGET_TABLE = {
    1: {
        "approach": {0: 2670, 1: 2149},
        "final":    {0: 2635, 1: 2189},
        "move_time": 0.68,
        "settle_time": 0.2,
    },
    2: {
        "approach": {0: 2340, 1: 2455},
        "final":    {0: 2300, 1: 2408},
        "move_time": 0.68,
        "settle_time": 0.2,
    },
    3: {
        "approach": {0: 2082, 1: 2950},
        "final":    {0: 2042, 1: 2910},
        "move_time": 0.68,
        "settle_time": 0.2,
    },
}

TRANSITION_TABLE = {}
