# Guidance-and-Control-of-an-Unmanned-Surface-Vehicle

## Project Summary

This repository implements an integrated guidance, trajectory generation, and path-following control system for an autonomous unmanned surface vehicle (USV) navigating through a minefield environment.

The system combines search-based path planning (A* / Hybrid A*), Bézier-curve trajectory smoothing, and a nonlinear path-following controller implemented in MATLAB and Simulink. Performance is evaluated through closed-loop simulation under increasing obstacle density.

## System Overview

The guidance and control pipeline is modular and structured as follows:

1. Minefield Definition
2. Initial Trajectory (A*/Hybrid A* Path Planning)
3. Trajectory Smoothing (Bezier Curves)
4. Nonlinear Path Following (Simulink)
5. USV Dynamics Simulation 

## Repository Structure    
    ├── MCM_v4.m
    │   ├── astar_path_planning_v1.m
    │   ├── HybridAstar_path_planning_v1_1.m
    │   ├── analysis_v2_1.m
    │   └── MinefieldAndWaypointPlotter_v2_1.m
    │
    ├── Scenario_PTF_path_following_v1.slx
    │
    └── plot_traversed_trajectory.m

## How to Run
1. Generate Trajectory
    * run MCM_v4.m
        * Adjust planner and smoothing parameters as needed 
2. Simulate closed-loop control
    * Open and run Scenario_PTF_path_following_v1.slx
    * Tune controller gains if desired
3. Visualize Results
   * run plot_traversed_trajector.m

## File Descriptions
### MCM_v4.m (Entry Point)
* Primary script for scenario set up and trajectory generation
* Initializes minefield, waypoints, and configures path planner
* Outputs dynamically feasible trajectory to the MATLAB workspace

#### Planner Selection
    use_astar = true; 
    use_hybridAStar = false;
or

    use_astar = false;
    use_hybridAStar = true;

#### Trajectory Smoothing
      use_smoothing = true; (uses smoothing)
      use_smoothing = false; (does not use smoothing)

#### Bezier Curve Degree
      n = 20 (default, change this value for a different degree of bezier curve)

### Scenario_PTF_path_following_v1.slx
* Simulink model implements:
    * USV dynamics
    * Nonlinear path following controller
* Uses trajectory generated in MCM_v4.m
* Controller gains are tunable for performance analysis

### plot_traversed_trajectory.m
* Post-processing and visualization
* Compares planned trajectory and actual traversed trajectory
* Overlays minefield  and waypoints 
    
### Controller Tuning Guidance
* All Controller parameters are accessible within SIMULINK
* Stability Constraints
    * K > 0
    * k4 > 0
    * 0 < k3 < pi/2
Gain tuning affects convergence speed and tracking accuracy

## Results Summary
* Navigate minefield with increasing mine density

<img width="631" height="507" alt="image" src="https://github.com/user-attachments/assets/f9fc52f1-963d-423d-8156-3240cd5b6601" />

* Hybrid A* reduces collisions to mines compared to standard A*

<img width="810" height="533" alt="image" src="https://github.com/user-attachments/assets/6f791fe2-451b-48f2-8b7c-c2df5af0d8ae" />

<img width="810" height="517" alt="image" src="https://github.com/user-attachments/assets/6923bc4e-62f5-40c9-9660-1be300377412" />



* Stable closed loop tracking achieved using nonlinear path following

* Demonstrates integration of planning, optimization, and control



