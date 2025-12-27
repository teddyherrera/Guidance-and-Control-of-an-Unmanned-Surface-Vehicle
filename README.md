# Guidance-and-Control-of-an-Unmanned-Surface-Vehicle

## Project Summary

This repository implements an integrated guidance, trajectory generation, and path-following control system for an autonomous unmanned surface vehicle (USV) navigating through a minefield environment.

The system combines search-based path planning (A / Hybrid A)**, Bézier-curve trajectory smoothing, and a nonlinear path-following controller implemented in MATLAB and Simulink. Performance is evaluated through closed-loop simulation under increasing obstacle density.

## System Overview

The guidance and control pipeline is modular and structured as follows:

Minefield Definition -> A* / Hybrid A* Path Planning -> Trajectory Smoothing (Bezier Curves) -> Nonlinear Path Following Controller (SIMULINK) -> USV Dynamics Simulation 

## Repository Structure
MCM_v4.m
    - astar_path_planning_v1.m
    - HybridAstar_path_planning_v1_1.m
    - analysis_v2_1.m
    - MinefieldAndWaypointPlotter_v2_1.m

Scenario_PTF_path_following_v1.slx

plot_traversed_trajectory.m

## How to Run
1. Generate Trajectory
    * run MCM_v4.m
        * Adjust planner and smoothing parameters as needed 
2. Simulate closed-loop control
    * Open and run Scenario_PTF_path_following_v1.slx
    * Tune controller gains if desired
3. Visualize Results
   * run plot_traversed_trajector.m
