import rclpy
import time
import numpy as np
import os
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controllers.IK import inverse_kinematics
from controllers.trajectory_utils import TrajectoryExecutor
from controllers.Jacobian import compute_trajectory

class Controller_4_1(Node):
    def __init__(self):
        super().__init__('trajectory_controller')

        # Add gripper position to HOME configuration
        self._HOME = [np.deg2rad(0), np.deg2rad(0),
                      np.deg2rad(0), np.deg2rad(0), 0.0]  # Added gripper position
        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        
        # Default gripper position when not specified
        self._default_gripper_pos = 0.0
        
        # Initialize trajectory executor
        self.trajectory_executor = TrajectoryExecutor(self)

    # Define function to get the home position
    def get_home_position(self):
        return self._HOME

    # Define function to move the robot to a certain joint position
    def move_to_joint_position(self, joint_positions):
        # Ensure we have 5 joint positions (add gripper if missing)
        if len(joint_positions) == 4:
            joint_positions = list(joint_positions) + [self._default_gripper_pos]
        
        # Convert all elements to float
        joint_positions = [float(pos) for pos in joint_positions]

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Add joint names (modify according to your robot)
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "gripper"]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()  # 1 second move duration

        msg.points = [point]
        self._publisher.publish(msg)
        self.get_logger().info(f"Moving to joint positions: {joint_positions}")
    
    def execute_circle_trajectory(self, center_x, center_y, center_z, radius, num_points=100, delay=0.2, gripper_pos=None):
        """
        Move the robot in a circular trajectory on a virtual wall
        
        Args:
            wall_distance: distance to the virtual wall (Y coordinate) in mm
            center_x: X coordinate of circle center on the wall
            center_z: Z coordinate of circle center on the wall (height)
            radius: radius of the circle in mm
            num_points: number of points to generate on the circle
            delay: delay between points in seconds
            gripper_pos: gripper position (optional)
        """
        # Set gripper position if provided
        gripper_position = self._default_gripper_pos if gripper_pos is None else gripper_pos
        
        self.get_logger().info(f"Starting circular trajectory on wall at Y={center_y}mm, center at X={center_x}mm, Z={center_z}mm with radius {radius}mm")
        
        # First move to home position
        time.sleep(1)
        
        # Generate points on the circle in the XZ plane (vertical wall)
        angles = np.linspace(0, 2*np.pi, num_points)
        
        for angle in angles:
            # Calculate the point on the circle
            x = center_x + radius * np.cos(angle)
            y = center_y   # Fixed distance to the wall
            z = center_z + radius * np.sin(angle)
            
            self.get_logger().info(f"Target position: ({x}, {y}, {z})")
            
            # Calculate inverse kinematics
            ik_result = inverse_kinematics(x, y, z)
            
            if isinstance(ik_result, str):
                # Position out of reach
                self.get_logger().warn(f"Position ({x}, {y}, {z}) is out of reach: {ik_result}")
                continue
            
            # Extract joint angles (ignoring the first element which is "Elbow up:" or "Elbow down:")
            _,theta1, theta2, theta3, theta4 = ik_result
            
            # Convert degrees to radians for the controller
            theta1_rad = np.radians(theta1)
            theta2_rad = np.radians(theta2)
            theta3_rad = np.radians(theta3)
            theta4_rad = np.radians(theta4)
            
            # Move to the calculated position (including gripper)
            self.move_to_joint_position([theta1_rad, theta2_rad, theta3_rad, theta4_rad, gripper_position])
            time.sleep(delay)
        
    def pick_and_place_sequence(self):
        """
        Performs a complete pick and place sequence using CSV trajectory data:
        1. Pick object from location A
        2. Place at location B
        3. Return to home
        4. Pick object from location B
        5. Place back at location A
        6. Return to home
        """
        # Path to trajectory files
        trajectories_dir = os.path.join(os.path.dirname(__file__), './trajectories')
        
        # Define trajectory files
        a_to_b_trajectory = os.path.join(trajectories_dir, 'pick_a_place_b.csv')
        b_to_a_trajectory = os.path.join(trajectories_dir, 'pick_b_place_a.csv')
        
        try:
            # Move to home position
            self.get_logger().info("Moving to home position")
            self.move_to_joint_position(self.get_home_position())
            time.sleep(1)
            
            # Execute A to B trajectory
            self.get_logger().info("Executing A to B pick and place trajectory")
            a_to_b = self.trajectory_executor.load_trajectory_from_csv(a_to_b_trajectory)
            if not a_to_b:
                self.get_logger().error(f"Failed to load trajectory from {a_to_b_trajectory}")
                return
            self.trajectory_executor.execute_trajectory(a_to_b)
            
            # Execute B to A trajectory
            self.get_logger().info("Executing B to A pick and place trajectory")
            b_to_a = self.trajectory_executor.load_trajectory_from_csv(b_to_a_trajectory)
            if not b_to_a:
                self.get_logger().error(f"Failed to load trajectory from {b_to_a_trajectory}")
                return
            self.trajectory_executor.execute_trajectory(b_to_a)
            
            self.get_logger().info("Pick and place sequence completed successfully!")
            
        except Exception as e:
            self.get_logger().error(f"Error during pick and place sequence: {str(e)}")
            
            # Try to return to home position for safety
            try:
                self.move_to_joint_position(self.get_home_position())
            except:
                self.get_logger().error("Failed to return to home position after error")

    def berry_pick_and_place(self):
        """
        Pick a berry and place it in a cup without squeezing it.
        Uses trajectory from CSV file with gentle gripper control.
        """
        # Path to trajectory file
        trajectories_dir = os.path.join(os.path.dirname(__file__), './trajectories')
        berry_trajectory = os.path.join(trajectories_dir, 'berry_pick_place.csv')
        
        try:
            # Move to home position
            self.get_logger().info("Moving to home position")
            self.move_to_joint_position(self.get_home_position())
            time.sleep(1)
            
            # Execute berry pick and place trajectory
            self.get_logger().info("Executing berry pick and place trajectory")
            berry_traj = self.trajectory_executor.load_trajectory_from_csv(berry_trajectory)
            if not berry_traj:
                self.get_logger().error(f"Failed to load trajectory from {berry_trajectory}")
                return
                
            # Use slightly more steps for smoother motion with the delicate berry
            self.trajectory_executor.execute_trajectory(berry_traj, time_scale=1.2, interpolation_steps=15)
            
            self.get_logger().info("Berry pick and place sequence completed successfully!")
            
        except Exception as e:
            self.get_logger().error(f"Error during berry pick and place: {str(e)}")
            
            # Try to return to home position for safety
            try:
                self.move_to_joint_position(self.get_home_position())
            except:
                self.get_logger().error("Failed to return to home position after error")

    def wipe_spill_sequence(self):
        """
        Picks up a sponge and wipes away a tea spill using CSV trajectory data.
        """
        # Path to trajectory files
        trajectories_dir = os.path.join(os.path.dirname(__file__), './trajectories')
        
        # Define trajectory files
        pickup_trajectory = os.path.join(trajectories_dir, 'sponge_pickup.csv')
        wipe_trajectory = os.path.join(trajectories_dir, 'spill_wipe.csv')
        return_trajectory = os.path.join(trajectories_dir, 'sponge_return.csv')
        
        # Move to home position
        self.get_logger().info("Moving to home position")
        self.move_to_joint_position(self.get_home_position())
        time.sleep(1)
        
        # Execute sponge pickup trajectory
        self.get_logger().info("Executing sponge pickup trajectory")
        pickup = self.trajectory_executor.load_trajectory_from_csv(pickup_trajectory)
        self.trajectory_executor.execute_trajectory(pickup)
        
        # Execute wiping trajectory
        self.get_logger().info("Executing wiping trajectory")
        wipe = self.trajectory_executor.load_trajectory_from_csv(wipe_trajectory)
        self.trajectory_executor.execute_trajectory(wipe)
        
        # Execute return trajectory
        self.get_logger().info("Executing return trajectory")
        return_traj = self.trajectory_executor.load_trajectory_from_csv(return_trajectory)
        self.trajectory_executor.execute_trajectory(return_traj)
        
        # Return to home position
        self.get_logger().info("Wiping sequence complete! Returning to home position.")
        self.move_to_joint_position(self.get_home_position())
        time.sleep(1)

    def Jacobian_velocity_traj(self, initial_angles_deg, v_desired, delta_t=0.1, num_steps=100, gripper_pos=0.0):
        """
        Computes and executes a trajectory using Jacobian velocity control.
        
        Args:
            initial_angles_deg (list): Initial joint angles in degrees (robot convention)
            v_desired (list): Desired end-effector velocity in mm/s [vx, vy, vz]
            delta_t (float): Time step for simulation in seconds
            num_steps (int): Maximum number of trajectory steps to compute
            gripper_pos (float): Gripper position to maintain throughout
        """
        self.get_logger().info(f"Computing Jacobian trajectory with v=[{v_desired[0]}, {v_desired[1]}, {v_desired[2]}]")
        self.get_logger().info(f"Initial angles (deg, robot convention): {initial_angles_deg}")
        
        # Compute trajectory using Jacobian
        time_steps, robot_angles, positions = compute_trajectory(
            np.array(initial_angles_deg), np.array(v_desired), delta_t, num_steps
        )
        
        # Check if we received valid data
        if len(time_steps) < 2 or len(robot_angles) < 2:
            self.get_logger().error("Insufficient trajectory points computed!")
            return None
        
        # Debug info
        self.get_logger().info(f"Trajectory has {len(time_steps)} points")
        self.get_logger().info(f"First position: {positions[0]}, Last: {positions[-1]}")
        self.get_logger().info(f"First angles: {robot_angles[0]}, Last: {robot_angles[-1]}")
        
        # Manual execution of trajectory for better control
        self.get_logger().info(f"Executing Jacobian trajectory with {len(time_steps)} points")
        
        for i in range(len(time_steps)):
            # Get joint angles for this step
            joint_angles_deg = robot_angles[i]
            
            # Log every 5th point
            if i % 5 == 0 or i == len(time_steps)-1:
                self.get_logger().info(f"Point {i}/{len(time_steps)-1}: angles={joint_angles_deg}")
            
            # Convert to radians
            joint_angles_rad = [np.radians(angle) for angle in joint_angles_deg]
            
            # Add gripper position
            joint_angles_cmd = joint_angles_rad + [gripper_pos]
            
            # Send to robot
            self.move_to_joint_position(joint_angles_cmd)
            
            # Wait appropriate amount of time
            if i < len(time_steps) - 1:
                wait_time = time_steps[i+1] - time_steps[i]
                time.sleep(wait_time)
        
        self.get_logger().info("Jacobian trajectory completed")
        return positions[-1]


def main(args=None):
    rclpy.init(args=args)
    controller = Controller_4_1()

    # Move to starting position
    controller.move_to_joint_position(controller.get_home_position())
    time.sleep(1)

    # controller.execute_circle_trajectory(center_x=0, center_y=200, center_z=150, radius=50, num_points=100, delay=0.05)
    # controller.wipe_spill_sequence()
    # controller.pick_and_place_sequence()
    # controller.berry_pick_and_place()
    
    
    initial_angles = [0, 30, 0, 0]  # Initial joint angles in degrees
    velocity = [-10, 0, 0]  # 20mm/s in X direction (increased for visibility)
    
    controller.get_logger().info("Starting Jacobian velocity trajectory")
    final_pos = controller.Jacobian_velocity_traj(
        initial_angles, 
        velocity, 
        delta_t=0.2,  # Larger time step
        num_steps=150,  # Fewer steps for testing
        gripper_pos=0.0
    )
    
    controller.get_logger().info(f"Final position: {final_pos}")
    
    
    # Return to home
    controller.move_to_joint_position(controller.get_home_position())

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
