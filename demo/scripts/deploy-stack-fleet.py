#!/usr/bin/env python3
"""
Deploy per-vehicle stacks with staggered timing to avoid thundering herd.

Usage:
    NUM_AGENTS=20 python3 demo/scripts/deploy-stack-fleet.py
    NUM_AGENTS=20 DEPLOY_STAGGER=1.0 python3 demo/scripts/deploy-stack-fleet.py
"""
import os
import sys
import time
import subprocess

NUM_AGENTS = int(os.environ.get('NUM_AGENTS', 3))
DEPLOY_STAGGER = float(os.environ.get('DEPLOY_STAGGER', 0.5))
STACKS_DIR = os.path.dirname(__file__) + '/../stacks'


def main():
    print(f"Deploying to {NUM_AGENTS} vehicles with {DEPLOY_STAGGER}s stagger...")

    for i in range(1, NUM_AGENTS + 1):
        stack_file = f"{STACKS_DIR}/gap_follower_racecar{i}.json"
        vehicle = f"racecar{i}"

        if not os.path.exists(stack_file):
            print(f"Warning: {stack_file} not found, skipping {vehicle}")
            continue

        print(f"[{i}/{NUM_AGENTS}] Deploying to {vehicle}...")

        # Call existing deploy-stack.py for single vehicle
        subprocess.run([
            sys.executable,
            os.path.dirname(__file__) + '/deploy-stack.py',
            stack_file,
            '--vehicle', vehicle
        ], check=True)

        # Stagger between deployments (except after last)
        if i < NUM_AGENTS and DEPLOY_STAGGER > 0:
            print(f"    Waiting {DEPLOY_STAGGER}s...")
            time.sleep(DEPLOY_STAGGER)

    print(f"\nDeployment complete!")


if __name__ == '__main__':
    main()
