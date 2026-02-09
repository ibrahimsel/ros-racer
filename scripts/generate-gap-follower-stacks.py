#!/usr/bin/env python3
"""
Generate gap_follower stack JSON files with varied parameters for N racecars.

Usage:
    NUM_AGENTS=20 python3 scripts/generate-gap-follower-stacks.py
    # Creates: demo/stacks/gap_follower_racecar1.json ... gap_follower_racecar20.json
"""
import os
import json
import random

NUM_AGENTS = int(os.environ.get('NUM_AGENTS', 20))
OUTPUT_DIR = 'demo/stacks'

# Driving style presets
STYLES = [
    ('aggressive',    {'max_speed': (2.5, 3.5), 'angle_gain': (1.5, 2.0), 'safe_gap': (0.8, 1.2)}),
    ('balanced',      {'max_speed': (1.8, 2.5), 'angle_gain': (1.2, 1.7), 'safe_gap': (1.2, 1.8)}),
    ('conservative',  {'max_speed': (1.2, 1.8), 'angle_gain': (0.8, 1.2), 'safe_gap': (1.8, 2.5)}),
]


def generate_stack(index: int) -> dict:
    """Generate stack JSON with env vars for driving style."""
    random.seed(index)  # Reproducible randomness per car

    # Cycle: racecar1=aggressive, racecar2=balanced, racecar3=conservative, ...
    style_name, style_ranges = STYLES[(index - 1) % len(STYLES)]

    max_speed = round(random.uniform(*style_ranges['max_speed']), 2)
    angle_gain = round(random.uniform(*style_ranges['angle_gain']), 2)
    safe_gap = round(random.uniform(*style_ranges['safe_gap']), 2)

    return {
        "metadata": {
            "name": f"gap_follower_{style_name}",
            "version": "1.0.0",
            "description": f"Gap follower with {style_name} driving style for racecar{index}"
        },
        "launch": {
            "url": "http://artifact-server:9090/gap_follower_balanced.tar.gz"
        },
        "env": {
            "GAP_FOLLOWER_MAX_SPEED": str(max_speed),
            "GAP_FOLLOWER_ANGLE_GAIN": str(angle_gain),
            "GAP_FOLLOWER_SAFE_GAP": str(safe_gap),
            "GAP_FOLLOWER_MIN_SPEED": "1.0"
        }
    }


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for i in range(1, NUM_AGENTS + 1):
        stack = generate_stack(i)
        filepath = f"{OUTPUT_DIR}/gap_follower_racecar{i}.json"
        with open(filepath, 'w') as f:
            json.dump(stack, f, indent=2)
        style_name = STYLES[(i - 1) % len(STYLES)][0]
        print(f"Generated {filepath} ({style_name})")

    print(f"\nTo deploy all: python3 demo/scripts/deploy-stack-fleet.py")


if __name__ == '__main__':
    main()
