# Kinodynamic Path Planning 

This path planning consider the robot's current state as well as kinodynamic
constraints of the robot to select the path to the goal.

- Inspired by [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
- It modifies the heusteric in A* Algorithm to give a cost to the kinodynamic
change in the robot from current state to next state.
- An OpenCV based visualization is also available

### Output (Simulated in OpenCV)
![output](output.jpg)
 
### Motion Planning Framework for [Team ARES](https://github.com/teamares)
![output](output.gif)
