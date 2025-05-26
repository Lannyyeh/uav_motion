# UAV_Motion

## Readme

- This repository is forked from [UAV-Motion](https://github.com/zhengyuxiang/UAV-Motion) and modified by Lanny Yeh. Please refer to the original repository and the [original README file](./README_original.md) for more details.

## Update Record

### 2025-05-26

- Procedure
  - Add complete docking procedure. Could achieve basic functions.
  - Add sliding window filter for external force estimation.

### 2025-05-20

- Procedure
  - Remove the unused state of 'INTERMEDIATE_STATE' and 'WAITING_TO_RELEASE'.
  - Add a state publisher. The waypoint generator now send waypoint msgs when the flight state is 'TRACKING', instead of the mavros status 'OFFBOARD'.
- Dropping judgement
  - Add position information for aiding.

### 2025-05-12

- The project reached a baseline.
  - The position controller is modified to a Adaptive Sliding Mode Controller (ASMC) and a mass estimation is added into it. It can reach better altitude tracking.
  - The command loop is revised and added more modes. Mostly for releasing from the KUKA Robot Arm. Add release service, dropping detection and some more corresponding functions for it.
  - Add a trajectory generator that can read waypoints from a txt file and send one waypoint at a time automatically. It matches our requirement better. This requires modification of waypoint action and service.
  - Add disturbance observer. It makes tracking performance better.
