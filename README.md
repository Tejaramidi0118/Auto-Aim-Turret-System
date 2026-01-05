# ROS Based Auto Aim Turret System

Overview
This project implements a real time auto aim turret using ROS. The system processes live camera input to detect a target and automatically adjusts servo motors to track it. The goal of the project is to demonstrate real time software control, modular system design, and ROS based communication.

System Architecture
The system is built using multiple ROS nodes. One node handles live video streaming from the camera. Another node processes the visual data and computes the required angles. A control node receives these angles and actuates the turret using servo motors. The nodes communicate using ROS publisher and subscriber mechanisms.

Key Engineering Concepts
The project uses a modular ROS node architecture to separate concerns and improve maintainability. A real time control loop is implemented to ensure continuous tracking. Debugging and tuning were performed to maintain stable and responsive motion. The system demonstrates integration of perception and control in a distributed software setup.

Technologies Used
Python
ROS2
OpenCV
Servo Motor Control

How to Run
Start the ROS environment.
Launch the camera streaming node.
Run the angle computation node.
Run the turret control node to begin tracking.

Learning Outcomes
Improved understanding of real time systems.
Hands on experience with ROS communication.
Stronger skills in debugging and systems integration.
