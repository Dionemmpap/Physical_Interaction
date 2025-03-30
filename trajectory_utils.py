import csv
import time
import io
import numpy as np
from controllers.IK import inverse_kinematics

class TrajectoryExecutor:
    def __init__(self, robot_controller):
        """
        Initialize with a robot controller that has move_to_joint_position method
        """
        self.controller = robot_controller
        self.logger = robot_controller.get_logger()
    
    def load_trajectory_from_csv(self, file_path):
        """
        Load a trajectory from CSV file
        
        Format: time,x,y,z,gripper
        Supports comment lines starting with # or //
        """
        trajectory = []
        
        try:
            # Read the file and filter out comment lines
            with open(file_path, 'r') as file:
                # Get all lines from file
                lines = file.readlines()
                
                # Filter out comment lines (starting with # or //)
                filtered_lines = []
                for line in lines:
                    line = line.strip()
                    if not line.startswith('#') and not line.startswith('//'):
                        filtered_lines.append(line)
                
                # If no data remains after filtering, return empty
                if len(filtered_lines) <= 1:  # Only header or empty
                    self.logger.error(f"No valid data points found in {file_path}")
                    return []
                
                # Create a string for the CSV reader
                csv_data = io.StringIO('\n'.join(filtered_lines))
                
                # Read data with csv.DictReader
                reader = csv.DictReader(csv_data)
                for row in reader:
                    try:
                        point = {
                            'time': float(row['time']),
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'z': float(row['z']),
                            'gripper': float(row['gripper'])
                        }
                        trajectory.append(point)
                    except (ValueError, KeyError) as e:
                        self.logger.warn(f"Skipping invalid row: {row} - Error: {str(e)}")
                        continue
                
            self.logger.info(f"Successfully loaded {len(trajectory)} points from {file_path}")
            return trajectory
            
        except Exception as e:
            self.logger.error(f"Error loading trajectory from {file_path}: {str(e)}")
            return []
    
    def execute_trajectory(self, trajectory, time_scale=1.0, interpolation_steps=10):
        """
        Execute a loaded trajectory with linear interpolation between points
        
        Args:
            trajectory: List of waypoints (dicts with time,x,y,z,gripper)
            time_scale: Factor to speed up or slow down execution (default: 1.0)
            interpolation_steps: Number of steps between waypoints
        """
        if not trajectory:
            self.logger.error("Empty trajectory provided")
            return

        self.logger.info(f"Starting trajectory execution with {len(trajectory)} waypoints")
        
        # Store successful joint positions
        last_success_joints = None
        
        # Starting time
        start_time = time.time()
        
        # Iterate through trajectory waypoints
        for i in range(len(trajectory) - 1):
            start_point = trajectory[i]
            end_point = trajectory[i + 1]
            
            # Calculate time for this segment
            segment_duration = (end_point['time'] - start_point['time']) * time_scale
            
            # Generate interpolated points
            for step in range(interpolation_steps + 1):
                # Skip first point if not the first segment (to avoid duplicates)
                if step == 0 and i > 0:
                    continue
                    
                # Interpolation factor (0 to 1)
                t = step / interpolation_steps
                
                # Linear interpolation between waypoints
                x = start_point['x'] + t * (end_point['x'] - start_point['x'])
                y = start_point['y'] + t * (end_point['y'] - start_point['y'])
                z = start_point['z'] + t * (end_point['z'] - start_point['z'])
                gripper_pos = start_point['gripper'] + t * (end_point['gripper'] - start_point['gripper'])
                
                # Calculate inverse kinematics
                ik_result = inverse_kinematics(x, y, z)
                
                if isinstance(ik_result, str):
                    self.logger.warn(f"Cannot reach interpolated position: {ik_result}")
                    continue
                
                _, theta1, theta2, theta3, theta4 = ik_result
                joint_angles = [
                    np.radians(theta1),
                    np.radians(theta2),
                    np.radians(theta3),
                    np.radians(theta4),
                    gripper_pos
                ]
                
                # Move robot to position
                self.controller.move_to_joint_position(joint_angles)
                
                # Store successful position
                last_success_joints = joint_angles
                self.controller._last_joint_position = joint_angles
                
                # Calculate how long to wait
                step_time = segment_duration / interpolation_steps
                time.sleep(step_time)
                
        self.logger.info("Trajectory execution completed")
        return last_success_joints

    def execute_jacobian_trajectory(self, time_steps, robot_angles, gripper_pos=0.0, time_scale=1.0):
        """
        Execute a trajectory generated by Jacobian velocity control
        
        Args:
            time_steps: List of time values for each step
            robot_angles: List of joint angles at each time step in robot convention
            gripper_pos: Gripper position to maintain throughout trajectory
            time_scale: Factor to speed up or slow down execution
        """
        if not time_steps or not robot_angles or len(time_steps) != len(robot_angles):
            self.logger.error(f"Invalid Jacobian trajectory: time_steps={len(time_steps) if time_steps else 0}, robot_angles={len(robot_angles) if robot_angles else 0}")
            return

        self.logger.info(f"Starting Jacobian trajectory execution with {len(time_steps)} points")
        
        # Debug: Print first and last points
        self.logger.info(f"First angles: {robot_angles[0]}")
        self.logger.info(f"Last angles: {robot_angles[-1]}")
        
        # Execute each point in the trajectory
        for i in range(len(time_steps)):
            # Current point's joint angles
            current_angles = robot_angles[i]
            
            # Debug: Print every 10th point
            if i % 10 == 0 or i == len(time_steps)-1:
                self.logger.info(f"Executing point {i}/{len(time_steps)}: angles={current_angles}")
            
            # Convert to radians if needed
            if abs(current_angles[0]) < 6.28:  # If angles are in radians already
                joint_angles = list(current_angles)
            else:  # If angles are in degrees
                joint_angles = [np.radians(angle) for angle in current_angles]
            
            # Add gripper position to joint angles
            joint_angles = joint_angles + [gripper_pos]
            
            # Move robot to position
            self.controller.move_to_joint_position(joint_angles)
            
            # Calculate how long to wait before the next point
            if i < len(time_steps) - 1:
                step_duration = (time_steps[i+1] - time_steps[i]) * time_scale
                if step_duration > 0:
                    time.sleep(step_duration)
        
        self.logger.info("Jacobian trajectory execution completed")

