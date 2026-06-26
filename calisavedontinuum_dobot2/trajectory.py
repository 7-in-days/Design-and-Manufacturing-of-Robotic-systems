#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Trajectory generation utilities for smooth robot motion.
"""


def minimum_jerk_scalar(r: float) -> float:
    """
    Minimum jerk interpolation for smooth movement.
    시작과 끝에서 속도와 가속도가 0에 가까워지도록 함.
    
    Args:
        r: Normalized time (0~1)
    
    Returns:
        s: Interpolated value (0~1)
    """
    r = max(0.0, min(1.0, r))  # clamp to 0~1
    return 10 * r**3 - 15 * r**4 + 6 * r**5
