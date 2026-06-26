#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Scenario parsing utilities for mission sequence input.
"""

from typing import List


def parse_scenario(text: str) -> List[int]:
    """
    Parse scenario text into list of target numbers.
    
    Supports formats: "1-2-3", "1 2 3", "1,2,3", "123"
    
    Args:
        text: Scenario string
    
    Returns:
        List of target numbers (1, 2, 3 only)
    
    Raises:
        ValueError: Invalid format or target numbers
    """
    if not text.strip():
        raise ValueError("Empty scenario text")
    
    # Remove separators and split
    cleaned = text.replace('-', '').replace(' ', '').replace(',', '')
    
    try:
        targets = [int(c) for c in cleaned]
    except ValueError:
        raise ValueError("Non-numeric characters in scenario")
    
    # Validate targets
    for t in targets:
        if t not in [1, 2, 3]:
            raise ValueError(f"Invalid target number: {t} (must be 1, 2, or 3)")
    
    return targets
