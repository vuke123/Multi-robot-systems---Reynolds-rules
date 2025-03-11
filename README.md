# Swarm Robotics Simulation using Reynolds Rules and Consensus Protocol

## Overview
This project presents the **implementation and testing of swarm robotics behaviors** using **Reynolds rules** in a simulated 2D environment, with an extension to consensus-based coordination. The research explores the **control of a Crazyflie swarm** within the **ROS2 and Gazebo** simulation environment, transitioning from simple **flocking behaviors** to **formation control and rendezvous** using consensus algorithms.

### Key Features:
- **Swarm Behavior Based on Reynolds Rules**: Implementing separation, alignment, and cohesion for realistic flocking behaviors.
- **Consensus-Based Control**: Enabling robots to achieve formation and rendezvous in a decentralized manner.
- **Obstacle Avoidance Mechanism**: Utilizing ray-based repulsion for smooth obstacle navigation.
- **Multi-Agent Simulation**: Tested in ROS2 and **CrazySim**, ensuring real-time control and analysis.

---

## Simulation Framework
The system was implemented in two simulation environments:
1. **Stage Simulator (2D Simulation)**
   - Initial experiments focused on **basic Reynolds rules** for flocking behaviors.
   - Implemented fundamental rules: **separation, alignment, cohesion**, and **target navigation**.

2. **Gazebo & CrazySim (3D Simulation)**
   - Transitioned to a more realistic **3D UAV swarm simulation**.
   - Introduced **consensus-based formation control** and **rendezvous protocols**.
   - Simulated obstacle-aware formations and **communication topology variations**.

---

## Consensus-Based Formation and Rendezvous
To improve swarm coordination, **consensus-based control** was introduced:
- **Rendezvous Protocol**: Enabling multiple UAVs to meet at a common position using adjacency matrices.
- **Formation Control Strategy**: Implementing structured formations (square, line, triangle) using decentralized control.

### **Adjustments & Modifications**
- **Obstacle Avoidance**: Upgraded from simple right-angle turns to **repulsion-based dodging**.
- **Acceleration-Based Updates**: Ensuring smoother velocity transitions.
- **Consensus Algorithm Optimizations**: Fine-tuning communication and control loops for better formation stability.

---

## Technologies Used
- **ROS2** – Communication framework for multi-robot coordination.
- **Gazebo & Stage Simulator** – 3D and 2D simulation environments.
- **Python** – Implementation of control algorithms.
- **OpenCV & PyTorch** – Vision-based processing (future applications).
- **CrazySim** – Custom framework for UAV swarm control.

---

## Results & Findings
- Swarm behaviors **successfully adapted from 2D to 3D**.
- Consensus-based control **improved coordination efficiency**.
- Formation strategies demonstrated **scalability and adaptability**.
- Videos showcasing results: [!Straight line formation](demo.gif).

---
