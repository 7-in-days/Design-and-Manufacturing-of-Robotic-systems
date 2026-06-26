#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration lookup tables for continuum robot targets.

Contains pre-calibrated motion parameters for each target position.
All values are placeholders and must be updated after actual calibration.
"""

# Calibration lookup table (실험 후 실제 값으로 수정 필요 - placeholder values)
# target number -> approach position, final position, move_time, settle_time
# 주의: 아래 값은 임시 placeholder이며, 실제 실험 후 각 target별 motor position을 수정해야 함
TARGET_TABLE = {
    1: {
        "approach": {0: 3505, 1: 1854},
        "final":    {0: 3505, 1: 1854},
        "move_time": 0.5,
        "settle_time": 0.25,
    },
    2: {
        "approach": {0: 2600, 1: 2600},
        "final":    {0: 2600, 1: 2600},
        "move_time": 0.5,
        "settle_time": 0.25,
    },
    3: {
        "approach": {0: 2048, 1: 3350}, # 3번
        "final":    {0: 2048, 1: 3350},
        "move_time": 0.5,
        "settle_time": 0.3,
    },
}

# Transition table for smooth transitions between targets
# (previous_target, next_target) -> approach position, final position, move_time, settle_time
# Transition lookup은 이전 자세에 따른 와이어 slack, 마찰, 탄성 이력 때문에 
# 같은 target이라도 접근 방향에 따라 끝단 위치가 달라지는 현상을 보정하기 위해 사용
# 주의: 모든 transition을 처음부터 채울 필요는 없으며, 실패하는 transition만 추가해도 됨
# TRANSITION_TABLE = { 
#     (0, 1): {
#         "approach": {1: 1690, 2: 1860},
#         "final":    {1: 1760, 2: 1810},
#         "move_time": 0.55,
#         "settle_time": 0.25,
#     },
#     (1, 0): {
#         "approach": {1: 1210, 2: 2160},
#         "final":    {1: 1260, 2: 2110},
#         "move_time": 0.60,
#         "settle_time": 0.30,
#     },
#     # 다른 transition은 TARGET_TABLE로 fallback
# }

TRANSITION_TABLE = {}