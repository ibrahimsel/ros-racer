<!-- banner: can be a image or a large font-->
<h1 align="center" style="font-weight: bold; margin-top: 20px; margin-bottom: 20px;">ROS Racer Blueprint</h1>

<!-- blurb: shortest possible summary (one line max) -->
<h3 align="center" style="font-weight: bold; margin-top: 20px; margin-bottom: 20px;">An Eclipse SDV Blueprint for Multi-Agent ROS Racers.</h3>


<!-- badges: meaningful meta information (one line max), do NOT include anything immediately visible -->
<p align="center">
	<a href="#status"><img src="https://img.shields.io/badge/status-maintained-green.svg" alt="status: maintained"></a>
	<a href="https://github.com/eclipse-sdv-blueprints/ros-racer/issues"><img src="https://img.shields.io/github/issues/eclipse-sdv-blueprints/ros-racer.svg" alt="issues: NA"></a>
    <a href="#license-and-copyright"><img src="https://img.shields.io/github/license/eclipse-sdv-blueprints/ros-racer.svg"></a>
</p>

<!-- quick links: local links (one line max) -->
<!-- Link to the (most important) h2 chapters, but do NOT link to anything visible without scrolling -->
<p align="center">
  <a href="#getting-started">Getting Started</a> •
  <a href="#documentation">Documentation</a> •
  <a href="#support">Need Help?</a> •
</p>

<!-- separate h2 chapters with white space: <br> -->
<br>

## Introduction
The **Eclipse SDV Blueprints** project is a collaborative initiative led by Eclipse SDV members to bring the "software defined vehicle" concepts to life. A crucial aspect of each blueprint is to ensure users can easily reproduce it on their own. This requires well-written and highly clear documentation. Users can utilize blueprints as they are, for inspiration or as a foundation to customize and meet their specific needs.

The Eclipse SDV Blueprints project hosts a collection of blueprints that demonstrate the application of technologies developed within the projects of the Eclipse SDV working group (sdv.eclipse.org). This allows for showcasing the capabilities and features of the software provided by the Eclipse SDV working group, while also exploring collaboration opportunities and integration of these technologies.



**ROS Racer Blueprint** is a A ROS based showcase where multi-agent autonomous racers that run F1Tenth.org software are orchestrated and managed by an Eclipse SDV software stack.

<br>

#### TODO: NOVNC
---
## Table of Contents

- [Overview](#overview)
- [Use Case Description](#use-case-description)
- [Prerequisites](#prerequisites)
- [System Architecture](#system-architecture)
- [Step-by-Step Guide](#step-by-step-guide)
  - [1. Setting Up the Environment](#1-setting-up-the-environment)
  - [2. Preparing the Docker Images](#2-preparing-the-docker-images)
  - [3. Configuring Muto](#3-configuring-muto)
  - [4. Simulating Edge Devices](#4-simulating-edge-devices)
  - [5. Deploying a Stack](#5-deploying-a-stack)
  - [6. Monitoring and Introspection](#6-monitoring-and-introspection)
- [Conclusion](#conclusion)
- [Appendix](#appendix)
  - [A. Sample Stack Definition](#b-sample-stack-definition)

---

## Overview

This blueprint demonstrates how **Eclipse Muto** can be used to automate software deployment across a fleet of edge devices. We will simulate multiple edge devices using Docker containers, each running an instance of Composer. The use case involves deploying a ROS2 application to these simulated devices, showcasing Composer's capabilities in orchestrating deployments, managing stacks, and handling updates.

## Use Case Description

**Scenario**: A company operates a fleet of autonomous delivery robots. Each robot is an edge device that needs to run the latest navigation and delivery software. The company wants to automate the deployment process to ensure all robots are running the same software version, can receive updates seamlessly, and can be monitored centrally.


- Automate the deployment of the navigation software to each robot.
- Manage updates and rollbacks if necessary.
- Handle different deployment actions like starting, stopping, and applying configurations.
- Provide tools for monitoring and introspection.

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **Docker**: Installed and running.
- **Docker Compose**: Installed for orchestrating multiple containers.
- **ROS2 Humble**: Installed on the host machine (for building images).
- **Git**: For cloning repositories.
- **Python 3.8** or later.
- **Colcon**: For building ROS2 packages.
- **Rosdep**: For installing ROS2 dependencies

## System Architecture

- **Host Machine**: Used to build Docker images and run the simulation.
- **Docker Containers**: Each container simulates an edge device that runs Muto.

![System Architecture Diagram]()

## Step-by-Step Guide

### 1. Setting Up the Environment

#### 1.1. Install Required Software

Ensure that [Docker](https://docs.docker.com/engine/install/ubuntu/) is installed on your system.


#### 1.2. Clone the Muto Repositories

```bash
cd $HOME
mkdir -p muto/src
cd muto/src/
git clone https://github.com/eclipse-muto/agent.git
git clone https://github.com/ibrahimsel/messages.git
git clone https://github.com/ibrahimsel/core.git
git clone -b dev/compose https://github.com/ibrahimsel/composer.git
```

### 2. Preparing the Docker Images

We will create a Docker image that includes ROS2 and Muto.

#### 2.1. Build the Docker Image

Build the Docker image and tag it as `muto_edge_device`:

```bash
docker build -t muto_edge_device .
```

### 3. Configuring Muto

#### 3.1. Define the Stack to Deploy

Create a stack definition that includes the software you want to deploy. An example is provided in the Appendix.

### 4. Simulating Edge Devices

#### 4.1. Start the Docker Containers

Launch the containers using Docker Compose:

```bash
docker compose up -d  # Runs in detached mode
```

Verify that the containers are running:

```bash
docker ps
```

### 5. Deploying a Stack

#### 5.1. Set Up the Central Control Node

We will use the host machine as the central control node to send deployment actions to the edge devices.

#### 5.2. Install ROS2 on Host (If Not Already Installed)

Ensure that ROS2 humble is installed on your host machine.

#### 5.3. Send a Deployment Action

Use the `ros2` command-line tools to send a `start` action to the edge devices.

First, source the ROS2 setup script:

```bash
source /opt/ros/humble/setup.bash
```

Then, send the `start` action:

```bash
ros2 topic pub /muto/stack muto_msgs/MutoAction "{method: 'start', payload: '{\"value\": {\"stackId\": \"org.eclipse.muto.sandbox:example_stack\"}}'}" --once
```

**Note**: Replace `org.eclipse.muto.sandbox:example_stack` with the `stackId` from your stack definition.

#### 5.4. Verify the Deployment

Check the logs of the edge devices to verify that the deployment occurred:

```bash
docker logs edge_device_1
docker logs edge_device_2
```

You should see logs indicating that the Composer node received the action, processed the stack, and executed the deployment pipeline.

### 6. Monitoring and Introspection

#### 6.1. Use ROS2 Tools

Use ROS2 command-line tools to monitor topics and services:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /muto/composed_stack
```

---

## Conclusion

This blueprint demonstrates how Eclipse Muto can be used to automate software deployment across multiple edge devices in a fleet. By simulating edge devices with Docker containers, we showcased Muto's ability to receive deployment actions, manage stack definitions, and execute deployment pipelines seamlessly.

This setup can be extended to real-world edge devices by installing Muto on each device.

---

## Appendix

### A. Sample `pipeline.yaml`
If you create your own plugin, you need to add them where you want the plugin to be executed. An example could be found below (check the commented out line)

```yaml
pipelines:
  - name: start
    pipeline:
      - sequence:
          - plugin: ComposePlugin
            service: muto_compose
          - plugin: NativePlugin
            service: muto_native
        # - plugin: ExamplePlugin
        #   service: ros2_service_for_your_plugin
          - plugin: LaunchPlugin
            service: muto_start_stack
    compensation:
      - plugin: LaunchPlugin
        service: muto_kill_stack

  - name: kill
    pipeline:
      - sequence:
          - plugin: LaunchPlugin
            service: muto_kill_stack

  - name: apply
    pipeline:
      - sequence:
          - plugin: LaunchPlugin
            service: muto_apply_stack
```

### B. Creating and updating stacks
- replace `example-stack` with the stackId you desire (`edge-device-99`, `eloquent-fox` etc.)


#### Create stack
```bash
curl -XPUT -u "ditto:ditto" \ 
-H "Content-type: application/json" \ 
-d '{}' 'https://sandbox.composiv.ai/api/2/things/org.eclipse.muto.sandbox:example-stack'
```

#### Update stack
```bash
curl -XPUT -H "Content-type: application/json" -d '{
  "thingId": "org.eclipse.muto.sandbox:example-stack",
  "policyId": "org.eclipse.muto.sandbox:example-stack",
  "definition": "org.eclipse.muto:Stack:0.0.1",
  "features": {
    "stack": {
      "properties": {
        "name": "Example Stack",
        "context": "example-stack",
        "stackId": "org.eclipse.muto.sandbox:example-stack",
        "url": "https://github.com/ros2/demos.git",
        "branch": "humble",
        "args": {
          "use_sim_time": "false"
        },
        "source": {
          "ros": "/opt/ros/humble/setup.bash",
          "workspace": "install/setup.bash"
        },
        "on_start": "",
        "on_kill": "",
        "launch_description_source": "talker_listener.launch.py"
      }
    }
  }
}' 'https://ditto:ditto@sandbox.composiv.ai/api/2/things/org.eclipse.muto.sandbox:example-stack'
```

### Explaining stack fields
- `thingId` and `policyId`: A unique ID and policy for your stack
- `definition`: and indicator that this is a stack
- `name`: An arbitrary name for your stack
- `context`: An arbitrary description for the stack
- `stackId`: Same as `thingId`
- `url`: Your git repo url
- `branch`: The branch you want to checkout
- `args`: ROS2 launch time arguments. The valid format could be found under the [update stack](#update-stack) part
- `source`: Environments that needs to be sourced on the edge device to be able to run the program

Below are the related fields for launch. You either need to provide both `on_start` and `on_kill` fields, leaving the `launch_description_source` empty; or you need to provide a `launch_description_source` and leave `on_start` and `on_kill` empty
- `launch_description_source`: A ROS2 launch file (XML or Python) for introspecting the ROS runtime (If not provided, Muto tries to start the program with `on_start` and `on_kill` scripts.). This should be your go-to solution for most cases
- `on_start`: An executable file to execute when a start signal is sent 
- `on_kill`: An executable file to execute when a stop signal is sent

**Note:** Muto recursively searches the directories for the file name under the cloned repository so you don't have to provide full paths to files for the above 3 fields. They just need to exist within your repo

---

## Contributing

If you want to contribute bug reports or feature requests, please use *GitHub Issues*.
<br>

## License and Copyright

This program and the accompanying materials are made available under the terms of the EPL v2.0 which is available at
[eclipse.org](https://www.eclipse.org/legal/epl-2.0/)
