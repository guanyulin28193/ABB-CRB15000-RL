# Unity-Based Reinforcement Learning for Robotic Task Sequences

This repository contains the source code for the thesis "Reinforcement Learning Based Value Function Integration in Behavior Trees for Global Optimization of Robotic Task Sequences".

## Project Overview
This project implements a reinforcement learning approach for optimizing robotic task sequences using behavior trees and value function integration. The implementation is based on Unity ML-Agents framework.

## Dependencies and Setup
1. Unity (version [2023.2.19f1])
2. ML-Agents ([(https://github.com/martkartasev/ml-agents/tree/vf_save_better)])
3. Please ensure all packages in the requirements file are installed
4. Ensure the ML-Agents version matches exactly as specified for compatibility

### Important Note
This project uses a specific version of the ML-Agents package. Please use the exact version to avoid compatibility issues.

## Project Structure

### Scenes
The project contains 5 main scenes:

1. **Grasp**
   - Standalone training environment for grasping
   - Uses basic reinforcement learning approach

2. **GraspVF**
   - Enhanced grasping environment
   - Integrates insertion task's value function into training
   - Demonstrates value function integration approach

3. **Insertion**
   - Standalone training environment for insertion tasks

4. **BT-baseline**
   - Complete task sequence (grasp + insertion)
   - Outputs performance data in CSV format
   - Baseline implementation without value function integration

5. **BT-VF**
   - Complete task sequence (graspVF + insertion)
   - Outputs performance data in CSV format
   - Implements value function integration

### Scripts

1. **PlatformAgent.cs**
   - ML Agent script for basic grasping RL

2. **Grasp_vf_agent.cs**
   - Train grasping ML agent with value function integration
   - Reads and incorporates insertion value function into reward calculation

3. **AgentInsertion.cs**
   - ML Agent script for insertion tasks
   - Handles insertion-specific behaviors

4. **PenaltyColliders.cs**
   - Handles collision penalty calculations
   - Implements collision detection and response

5. **BT.cs**
   - Task switching for baseline implementation
   - Manages task sequence execution

6. **BTVF.cs**
   - Task switching with value function integration
   - Enhanced task sequence management

7. **ik.cs & ikgrpc.cs**
   - GRPC communication scripts
   - Interface with external Python-based inverse kinematics solver

## Data Output
- Both BT-baseline and BT-VF scenes output performance data in CSV format
- Data can be used for performance analysis and comparison

## External Dependencies
- Requires external Python environment for inverse kinematics calculations
- Uses GRPC for communication between Unity and Python
- Run ik_server.py file in ./python for ik

## Additional Notes
- Ensure proper setup of GRPC communication for inverse kinematics
- Check collision settings when running training scenarios
- Review value function integration parameters when using VF-enhanced scenes
- Make sure that each game object has the correct agent attached on it
