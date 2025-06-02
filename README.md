# SurveillanceBot: Autonomous Navigation and Mapping

**Robotics Assignment Report**  
School of Computer Science & Applied Mathematics  
University of the Witwatersrand

**Team Members:**
- Banzile Nhlebela (2571291)
- Daniel Ngobe (2556833)  
- Palesa Rapolaki (2550752)
- Mixo Khoza (2429356)

---

## Introduction

This report presents the implementation of a SurveillanceBot system for autonomous navigation and mapping using a TurtleBot robot in a simulated environment. The project encompasses three main components: environment mapping using gmapping, path planning using Rapidly-exploring Random Trees (RRT), and robot navigation using PID control.

## System Architecture

### Overview
The SurveillanceBot system consists of three integrated modules:
- **Mapping Module**: SLAM-based environment mapping using gmapping
- **Path Planning Module**: RRT algorithm for collision-free path generation  
- **Navigation Module**: PID-controlled robot movement with state-machine behavior

### Environment Setup
The robot operates in the provided environment. Gmapping was used to obtain the obstacle locations and then matplotlib was used to visualise the path planning algorithm in action on the map.

## Mapping Implementation

### SLAM Configuration
We utilized the gmapping package for Simultaneous Localization and Mapping (SLAM) with optimized parameters:

```bash
roslaunch turtlebot_gazebo gmapping_demo.launch \
slam_gmapping/linearUpdate:=0.2 \
slam_gmapping/angularUpdate:=0.2 \
slam_gmapping/temporalUpdate:=2.0 \
slam_gmapping/map_update_interval:=2.0 \
slam_gmapping/xmin:=-10.0 \
slam_gmapping/xmax:=10.0 \
slam_gmapping/ymin:=-10.0 \
slam_gmapping/ymax:=10.0 \
slam_gmapping/minimumScore:=1000 \
slam_gmapping/particles:=30 \
slam_gmapping/maxRange:=12.0 \
slam_gmapping/maxUrange:=8.0 \
slam_gmapping/srr:=0.1 \
slam_gmapping/srt:=0.2 \
slam_gmapping/str:=0.1 \
slam_gmapping/stt:=0.2
```

### Parameter Justification
- **minimumScore**: Set to 1000 for improved mapping accuracy in complex environments
- **particles**: 30 particles provide sufficient localization accuracy while maintaining computational efficiency
- **Update rates**: Linear and angular updates of 0.2 ensure frequent map updates during movement
- **Sensor parameters**: maxRange and maxUrange optimized for the TurtleBot's sensor capabilities

## Path Planning Algorithm

### RRT Implementation
We implemented a Rapidly-exploring Random Tree (RRT) algorithm for path planning, chosen over Probabilistic Roadmap (PRM) for the following reasons:

- Better exploration of narrow passages and constrained environments
- Single-query efficiency for one-time path planning scenarios
- Natural production of connected paths without additional post-processing
- Suitability for dynamic environments without pre-processing requirements

### Algorithm Details
The RRT algorithm incorporates:
- **Sampling Strategy**: 95% random sampling within map boundaries, 5% goal-biased sampling
- **Steering Function**: Limited step size of 5 units to ensure smooth path generation
- **Collision Detection**: 
  - Liang-Barsky algorithm for line-rectangle intersection
  - Ray-casting algorithm for point-in-polygon tests
  - Line-polygon intersection for angled obstacles
- **Goal Threshold**: 5-unit radius for goal reaching tolerance

## Robot Control System

### PID Control Implementation
The navigation system employs separate PID controllers for linear and angular motion.

**PID Gains**:
- Linear motion: Kₚ = 1.0, Kᵢ = 0.0, Kₐ = 0.1
- Angular motion: Kₚ = 2.0, Kᵢ = 0.0, Kₐ = 0.1

### Localization
Robot pose estimation utilizes Gazebo's model state service, providing:
- Position coordinates (x, y) from `/gazebo/model_states`
- Orientation conversion from quaternion to Euler angles
- Real-time pose feedback at 10 Hz control frequency

## System Integration

The complete navigation system follows this workflow:
1. Accept target coordinates as command-line arguments
2. Generate collision-free path using RRT algorithm
3. Execute sequential waypoint navigation using PID control
4. Implement turn-then-move behavior for each waypoint
5. Terminate upon reaching final destination within tolerance

## File Structure

```
├── map
    ├── map.txt
    └── map.pgm
├── scripts
    ├── turtlebot_pid.py
    ├── path_planning.py
    └── ...
├── src
    └── teleop_twist_keyboard
        ├── teleop_twist_keyboard.py
        └── ...
├── links.txt
├── report.pdf
├── startWorld
└── ...
```

## Usage

### Prerequisites
- ROS Kinetic
- TurtleBot simulation environment
- Required ROS packages: `gmapping`, `turtlebot_gazebo`

### Running the System

1. **Start the simulation environment**:
   ```bash
   ./startWorld
   ```

2. **Run the path planning and pid algo**:
   ```bash
   python scripts/turtlebot_pid.py target_x target_y
   ```

## Performance Characteristics

### Path Planning Efficiency
- Maximum iterations: 5,000 (typically converges in <1,000 iterations)
- Step size: 5 units (balance between exploration and smoothness)
- Success rate: High for feasible target locations within map boundaries

### Control System Stability
- Position accuracy: ±0.1 units
- Angular accuracy: ±0.05 radians (≈3 degrees)
- Convergence time: Variable based on path complexity and distance

## Key Achievements

- Robust obstacle avoidance in complex environments
- Efficient path planning with geometric and angled obstacles
- Stable robot control with predictable behavior
- Scalable system architecture suitable for real-world deployment

## Future Improvements

Future enhancements could incorporate:
- Dynamic obstacle avoidance
- Improved sensor fusion
- Adaptive PID tuning for varying environmental conditions
- Integration with computer vision for object detection

---

*This project was completed as part of the Robotics course at the University of the Witwatersrand, School of Computer Science & Applied Mathematics.*
