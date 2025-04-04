# Lynxmotion AL5A ROS2 Control System based on Edubot

This project is a comprehensive ROS2-based robot arm control system for the Lynxmotion AL5A robot arm. It implements inverse kinematics, Jacobian-based velocity control, and trajectory planning for various manipulation tasks.

## Getting Started

Please follow the following instructions to run the code:

1.  **Copy Files:**  
    Copy all project files to `edubot/python_impl/src/controllers`.

2.  **Build the Workspace:**  
    Navigate to your workspace directory containing the `edubot` package and build it:
    
    ```bash
    cd /path/to/your/ws/edubot/python_impl
    colcon build
    ```
    *(Replace `/path/to/your/ws/` with the actual path to your ROS2 workspace)*

3.  **Launch RViz Simulation:**  
    Open RViz with the robot model:
    
    ```bash
    ros2 launch edubot sim.launch.py
    ```

4.  **Source and Run Controller:**  
    Open a new terminal, source your workspace, and run the main controller node:
    
    ```bash
    
    source python_impl/install/setup.bash
    ros2 run controllers Controller_4_1
    ```

5.  **Select Trajectory:**  
    Follow the prompts in the terminal running `Controller_4_1` to select the desired trajectory or task to execute.

6.  **Observe:**  
    Sit back and watch the robot perform the selected task in RViz!

## System Architecture

The project follows this directory structure within `edubot/python_impl/src/controllers`:

```text
controllers/
├── __init__.py                    # Package initialization
├── Controller_4_1.py              # Main controller implementation
├── example_traj.py                # Example trajectory 
├── IK.py                          # Inverse kinematics module
├── IK_set_wrist_angle.py          # IK with fixed wrist angle
├── Jacobian.py                    # Basic Jacobian implementation
├── Jacobian_final.py              # Optimized Jacobian with joint limits
├── joint_verifier.py              # Tool for testing joint movements
├── trajectory_utils.py            # CSV file trajectory loading/execution
├── workspacedifferentapproach3d.py # Workspace visualization
└── trajectories/                  # CSV trajectory definitions
    ├── berry_pick_place.csv       # Berry handling trajectory
    ├── pick_a_place_b.csv         # Pick and place sequence A→B
    ├── pick_b_place_a.csv         # Pick and place sequence B→A
    ├── spill_wipe.csv             # Spill cleaning pattern
    └── sponge_return.csv          # Return sponge to original position
```

## Code Description

### Key Components

#### **Trajectory Files (CSV)**  
Located in the `trajectories/` directory, these files define waypoints for complex motions. Each row specifies:
- Time
- Target Cartesian coordinates (`x, y, z`)
- Gripper position

#### **Controller Script (`Controller_4_1.py`)**  
This is the main executable script that orchestrates various tasks and integrates supporting modules. It includes implementations for:
- Circle trajectory drawing
- Pick and place operations
- Gentle handling of delicate objects (e.g., berries)
- Complex wiping motions
- Jacobian-based velocity control demonstrations

#### **Supporting Modules**
- `IK.py`: Converts desired Cartesian end-effector coordinates into joint angles using inverse kinematics.
- `trajectory_utils.py`: Loads, parses, and executes trajectories defined in CSV files, including interpolation between waypoints.
- `Jacobian_final.py`: Implements Jacobian-based velocity control, including calculation of the Jacobian matrix, handling of kinematic singularities, and enforcement of joint limits.

## Implemented Tasks

The `Controller_4_1.py` script allows executing several predefined tasks:

### **Circle Trajectory**  
Draws circular patterns in 3D space. Example usage within the controller code:
```python
controller.execute_circle_trajectory(center_x=0, center_y=180, center_z=150, radius=80)
```

### **Pick and Place Sequence**  
Transfers objects between two predefined locations (A and B):
```python
controller.pick_and_place_sequence()
```

### **Berry Pick and Place**  
Demonstrates gentle handling suitable for delicate objects:
```python
controller.berry_pick_and_place()
```

### **Wipe Spill Sequence**  
Performs a cleaning operation using zig-zag and spiral patterns:
```python
controller.wipe_spill_sequence()
```

### **Jacobian Velocity Control**  
Moves the end-effector with a specified Cartesian velocity using the Jacobian matrix for control:
```python
controller.Jacobian_velocity_traj(initial_angles_deg=[60, -30, -30, 0], v_desired=[0, -10, 0])
```

## Creating Custom Trajectories

You can define your own manipulation tasks by creating CSV files in the `trajectories/` directory. The format requires the following columns:

```text
time, x, y, z, gripper
```

### **Example Waypoints:**
```text
0.0,  0, 150, 150, 0.0   # Start position, gripper open
1.0, 10, 160, 160, 0.0   # Move to point 1
2.5, 10, 160, 100, 0.0   # Move down
3.0, 10, 160, 100, 1.0   # Close gripper
4.0,  0, 150, 150, 1.0   # Move back to start area, holding object
```

**Column Descriptions:**
- `time`: Timestamp in seconds from the start of the trajectory.
- `x, y, z`: Target Cartesian coordinates of the end-effector in millimeters (mm).
- `gripper`: Target gripper position, ranging from `0.0` (fully open) to `1.0` (fully closed).

Lines starting with `#` or empty lines are ignored and can be used for comments and spacing. The `trajectory_utils.py` module handles loading and interpolating these waypoints.

## Development and Testing

### **Joint Verification**
You can test the movement range and response of individual joints using the `joint_verifier` tool:
```bash
ros2 run controllers joint_verifier
```
Follow the prompts to select a joint and target angle.

### **Workspace Visualization**
The `workspacedifferentapproach3d.py` script can visualize the robot's reachable workspace and test forward kinematics:
```bash
ros2 run controllers workspacedifferentapproach3d
