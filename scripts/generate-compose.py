#!/usr/bin/env python3
"""
Generate docker-compose.yml for N racecars with configurable RMW.

Usage:
    # Default: 3 cars with CycloneDDS
    python3 scripts/generate-compose.py > docker-compose.yml

    # 20 cars with Zenoh (recommended for 10+ agents)
    NUM_AGENTS=20 RMW_IMPLEMENTATION=rmw_zenoh_cpp python3 scripts/generate-compose.py > docker-compose.yml
"""
import os
import sys
import yaml

NUM_AGENTS = int(os.environ.get('NUM_AGENTS', sys.argv[1] if len(sys.argv) > 1 else 3))
RMW = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
USE_ZENOH = RMW == 'rmw_zenoh_cpp'


def generate_edge_service(index: int) -> dict:
    vehicle_name = f'racecar{index}'

    env = [
        'ROS_DOMAIN_ID=2',
        'ROS_LOCALHOST_ONLY=0',
        f'VEHICLE_NAME={vehicle_name}',
        f'RMW_IMPLEMENTATION={RMW}',
    ]

    depends_on = {}
    if USE_ZENOH:
        env.append('ZENOH_ROUTER_CHECK_ATTEMPTS=-1')
        depends_on['zenoh-router'] = {'condition': 'service_healthy'}

    service = {
        'build': {'context': '.', 'dockerfile': 'Dockerfile.edge'},
        'hostname': vehicle_name,
        'environment': env,
        'volumes': ['./demo:/edge/demo:ro'],
        'networks': ['x11'],
        'deploy': {
            'resources': {
                'limits': {'cpus': '0.5', 'memory': '512M'},
                'reservations': {'cpus': '0.25', 'memory': '256M'}
            }
        }
    }

    if depends_on:
        service['depends_on'] = depends_on

    return service


# Base services
services = {
    'artifact-server': {
        'image': 'nginx:alpine',
        'volumes': [
            './demo/artifacts:/usr/share/nginx/html:ro',
            './config/nginx.conf:/etc/nginx/nginx.conf:ro'
        ],
        'networks': ['x11'],
        'deploy': {'resources': {'limits': {'cpus': '1.0', 'memory': '256M'}}}
    },
    'novnc': {
        'image': 'theasp/novnc:latest',
        'environment': ['DISPLAY_WIDTH=1920', 'DISPLAY_HEIGHT=1080', 'RUN_XTERM=no', 'RUN_FLUXBOX=yes'],
        'ports': ['18080:8080'],
        'networks': ['x11']
    },
}

# Add zenoh-router only if using Zenoh RMW
if USE_ZENOH:
    services['zenoh-router'] = {
        'build': {'context': '.', 'dockerfile': 'Dockerfile.zenoh-router'},
        'hostname': 'zenoh-router',
        'networks': ['x11'],
        'healthcheck': {
            'test': ['CMD', 'nc', '-z', 'localhost', '7447'],
            'interval': '5s',
            'retries': 3,
            'start_period': '10s'
        }
    }

# Sim service
sim_env = [
    'DISPLAY=novnc:0.0',
    'ROS_DOMAIN_ID=2',
    'ROS_LOCALHOST_ONLY=0',
    f'RMW_IMPLEMENTATION={RMW}',
    f'NUM_AGENTS={NUM_AGENTS}'
]
sim_depends = {'novnc': {'condition': 'service_started'}}

if USE_ZENOH:
    sim_env.append('ZENOH_ROUTER_CHECK_ATTEMPTS=-1')
    sim_depends['zenoh-router'] = {'condition': 'service_healthy'}

# Scale sim resources based on number of agents
# Base: 2 CPU + 0.3 per agent, 2GB + 150MB per agent
sim_cpus = min(2.0 + (NUM_AGENTS * 0.3), 16.0)  # Cap at 16 CPUs
sim_memory = f"{min(2048 + (NUM_AGENTS * 150), 16384)}M"  # Cap at 16GB

services['sim'] = {
    'build': {'context': '.', 'dockerfile': 'Dockerfile.sim'},
    'volumes': ['.:/sim_ws/src/f1tenth_gym_ros'],
    'environment': sim_env,
    'depends_on': sim_depends,
    'networks': ['x11'],
    'deploy': {'resources': {'limits': {'cpus': str(sim_cpus), 'memory': sim_memory}}}
}

# Add edge services
for i in range(1, NUM_AGENTS + 1):
    service_name = 'edge' if i == 1 else f'edge{i}'
    services[service_name] = generate_edge_service(i)

config = {'services': services, 'networks': {'x11': {}}}

# Header comment
print(f"# Auto-generated for {NUM_AGENTS} racecars using {RMW}")
print(f"# Regenerate: NUM_AGENTS={NUM_AGENTS} RMW_IMPLEMENTATION={RMW} python3 scripts/generate-compose.py > docker-compose.yml")
print(yaml.dump(config, default_flow_style=False, sort_keys=False))
