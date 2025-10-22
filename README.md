# 🦾 Modeling and Simulation of 2-Link Planar Robotic Arm

### Department of Mechanical Engineering, IIT Ropar  
**Course:** Machine Design Laboratory-II (ME306)  
**Supervised by:** Dr. Srikant Sekhar Padhee  
**Team Members:** Jai Kumar Prajapati, Janhvi Soni, Kartikey, Kultaj Singh, L. G. Deepak  

---

## 🔍 Overview
This project focuses on the **modeling, simulation, and control** of a two-link planar robotic arm using an integrated workflow of **SolidWorks**, **MATLAB Simulink**, and **Simscape Multibody**.  
The objective is to design a planar manipulator capable of accurately tracking end-effector trajectories using **PID control**.

---

## ⚙️ Objectives
- To model a **2-link planar robotic arm** in SolidWorks.  
- To simulate its dynamic behavior using **Simscape Multibody**.  
- To implement **PID controllers** for accurate trajectory tracking.  
- To validate **inverse and forward kinematics** through MATLAB functions.

---

## 🧩 Methodology

### 1. Mechanical Design (SolidWorks)
- Base: Circular base (150 mm diameter, 20 mm thick) supporting the manipulator.  
- Links:  
  - Link 1: 100 mm long, 30 mm diameter cylindrical cross-section.  
  - Link 2: 80 mm long.  
- Revolute joints provided between base-link and link-link connections.  
- End-effector designed as a simple gripper for visualization.  

### 2. Export and Import
- The SolidWorks assembly was exported using the **Simscape Multibody Link Plugin** (XML + STEP).  
- Imported into MATLAB using the **Simscape Multibody Import Tool**, preserving geometry and joint data.

### 3. Control System (Simulink)
- Two **PID controllers** (one per joint) implemented for closed-loop trajectory tracking.  
- **Trajectory generation** functions created using MATLAB scripts to define circular, linear, and square paths.  
- **Inverse kinematics** converts desired end-effector position into joint angles.  
- **Forward kinematics** used to visualize and verify motion.  

### 4. Key MATLAB Functions
```matlab
% Example: Inverse Kinematics
function [theta1d,theta2d] = inverse_kinematics(xd,yd)
l1 = 1; l2 = 1;
theta2d = acos((xd^2 + yd^2 - l1^2 - l2^2)/(2*l1*l2));
theta1d = atan(yd/xd) - atan((l2*sin(theta2d))/(l1 + l2*cos(theta2d)));
end

