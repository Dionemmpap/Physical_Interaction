import rclpy
import time
import numpy as np
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from controllers.IK_set_wrist_angle import inverse_kinematics
from controllers.IK import inverse_kinematics

class Controller_4_1(Node):
    def __init__(self):
        super().__init__('circular_trajectory_controller')

        # Add gripper position to HOME configuration
        self._HOME = [np.deg2rad(0), np.deg2rad(0),
                      np.deg2rad(0), np.deg2rad(0), 0.0]  # Added gripper position
        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        
        # Default gripper position when not specified
        self._default_gripper_pos = 0.0

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
        # self.get_logger().info("Moving to home position")
        # self.move_to_joint_position(self._HOME)
        time.sleep(1)
        
        # Generate points on the circle in the XZ plane (vertical wall)
        angles = np.linspace(0, 2*np.pi, num_points)
        
        for angle in angles:
            # Calculate the point on the circle
            # For a circle on a vertical wall:
            # - Y remains constant (distance to the wall)
            # - X and Z vary to create the circle
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
            # theta1 = ik_result[0][1]
            # theta2 = ik_result[0][2]
            # theta3 = ik_result[0][3]
            # theta4 = ik_result[0][4]

            
            # Convert degrees to radians for the controller
            theta1_rad = np.radians(theta1)
            theta2_rad = np.radians(theta2)
            theta3_rad = np.radians(theta3)
            theta4_rad = np.radians(theta4)
            
            # Move to the calculated position (including gripper)
            self.move_to_joint_position([theta1_rad, theta2_rad, theta3_rad, theta4_rad, gripper_position])
            time.sleep(delay)
        
        # Return to home position
        # self.get_logger().info("Returning to home position")
        # self.move_to_joint_position(self._HOME)

    def pick_and_place_sequence(self):
        """
        Performs a complete pick and place sequence:
        1. Pick object from location A
        2. Place at location B
        3. Return to home
        4. Pick object from location B
        5. Place back at location A
        6. Return to home
        
        Uses all joints with diverse motions for demonstration.
        """
        # Define locations (10cm apart)
        loc_a = {"x": -60, "y": 200, "z": 120 }  # directly in front
        loc_b = {"x": 50, "y": 200, "z": 120}  # 10cm to the right
        
        # Approach height (20mm above objects)
        approach_height = 50
        
        self.get_logger().info("Starting pick and place sequence")
        
        # --- FIRST SEQUENCE: A to B ---
        
        # 1. Move to home position
        self.get_logger().info("Moving to home position")
        self.move_to_joint_position(self.get_home_position())
        time.sleep(1)
        
        # 2. Move to position above object A with gripper open
        self.get_logger().info(f"Moving above location A: ({loc_a['x']}, {loc_a['y']}, {loc_a['z'] + approach_height})")
        ik_result = inverse_kinematics(loc_a["x"], loc_a["y"], loc_a["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach above A: {ik_result}")
            return
        
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper open
        ])
        time.sleep(0.5)
        
        # 3. Lower to grasp position
        self.get_logger().info(f"Lowering to grasp at location A: ({loc_a['x']}, {loc_a['y']}, {loc_a['z']})")
        ik_result = inverse_kinematics(loc_a["x"], loc_a["y"], loc_a["z"])
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach grasp position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper open
        ])
        time.sleep(1)
        
        # 4. Close gripper
        self.get_logger().info("Closing gripper")
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.8  # Gripper closed
        ])
        time.sleep(0.5)
        
        # 5. Lift object
        self.get_logger().info(f"Lifting object from location A")
        ik_result = inverse_kinematics(loc_a["x"], loc_a["y"], loc_a["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach lift position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.8  # Gripper closed
        ])
        time.sleep(0.5)
        
        # 6. Move to intermediate position (arc motion using joint 1)
        self.get_logger().info("Moving through arc path to location B")
        ik_result = inverse_kinematics(50, 180, loc_a["z"] + 100)  # Higher arc midpoint
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach midpoint: {ik_result}")
        else:
            _, theta1, theta2, theta3, theta4 = ik_result
            self.move_to_joint_position([
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.8  # Gripper closed
            ])
            time.sleep(0.5)
        
        # 7. Move above location B
        self.get_logger().info(f"Moving above location B: ({loc_b['x']}, {loc_b['y']}, {loc_b['z'] + approach_height})")
        ik_result = inverse_kinematics(loc_b["x"], loc_b["y"], loc_b["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach above B: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.8  # Gripper closed
        ])
        time.sleep(0.5)
        
        # 8. Lower to place position
        self.get_logger().info(f"Lowering to place at location B: ({loc_b['x']}, {loc_b['y']}, {loc_b['z']})")
        ik_result = inverse_kinematics(loc_b["x"], loc_b["y"], loc_b["z"])
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach place position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.8  # Gripper closed
        ])
        time.sleep(0.5)
        
        # 9. Open gripper
        self.get_logger().info("Opening gripper")
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper open
        ])
        time.sleep(0.5)
        
        # 10. Lift from object
        self.get_logger().info("Lifting from location B")
        ik_result = inverse_kinematics(loc_b["x"], loc_b["y"], loc_b["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach lift position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper open
        ])
        time.sleep(0.5)
        
        # 11. Return to home position
        self.get_logger().info("Returning to home position")
        self.move_to_joint_position(self.get_home_position())
        time.sleep(1)
        
        # --- SECOND SEQUENCE: B to A ---
        
        # 12. Move to position above object B with gripper open
        self.get_logger().info(f"Moving above location B: ({loc_b['x']}, {loc_b['y']}, {loc_b['z'] + approach_height})")
        ik_result = inverse_kinematics(loc_b["x"], loc_b["y"], loc_b["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach above B: {ik_result}")
            return
        
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper open
        ])
        time.sleep(0.5)
        
        # 13. Lower to grasp position
        self.get_logger().info(f"Lowering to grasp at location B")
        ik_result = inverse_kinematics(loc_b["x"], loc_b["y"], loc_b["z"])
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach grasp position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper open
        ])
        time.sleep(0.5)
        
        # 14. Close gripper
        self.get_logger().info("Closing gripper")
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.8  # Gripper closed
        ])
        time.sleep(0.5)
        
        # 15. Lift object
        self.get_logger().info(f"Lifting object from location B")
        ik_result = inverse_kinematics(loc_b["x"], loc_b["y"], loc_b["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach lift position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.8  # Gripper closed
        ])
        time.sleep(0.5)
        
        # 16. Move to intermediate position (different path for variety)
        self.get_logger().info("Moving through different arc path to location A")
        ik_result = inverse_kinematics(50, 180, loc_a["z"] + 70)  # Higher and farther back
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach midpoint: {ik_result}")
        else:
            _, theta1, theta2, theta3, theta4 = ik_result
            self.move_to_joint_position([
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.8  # Gripper closed
            ])
            time.sleep(0.5)
        
        # 17. Move above location A
        self.get_logger().info(f"Moving above location A: ({loc_a['x']}, {loc_a['y']}, {loc_a['z'] + approach_height})")
        ik_result = inverse_kinematics(loc_a["x"], loc_a["y"], loc_a["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach above A: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.8  # Gripper closed
        ])
        time.sleep(0.5)
        
        # 18. Lower to place position
        self.get_logger().info(f"Lowering to place at location A: ({loc_a['x']}, {loc_a['y']}, {loc_a['z']})")
        ik_result = inverse_kinematics(loc_a["x"], loc_a["y"], loc_a["z"])
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach place position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.8  # Gripper closed
        ])
        time.sleep(0.5)
        
        # 19. Open gripper
        self.get_logger().info("Opening gripper")
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper open
        ])
        time.sleep(0.5)
        
        # 20. Lift from object
        self.get_logger().info("Lifting from location A")
        ik_result = inverse_kinematics(loc_a["x"], loc_a["y"], loc_a["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach lift position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        self.move_to_joint_position([
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper open
        ])
        time.sleep(0.5)
        
        # 21. Return to home position
        self.get_logger().info("Returning to home position - sequence complete")
        self.move_to_joint_position(self.get_home_position())
        time.sleep(1)
        
        self.get_logger().info("Pick and place sequence completed successfully!")

    def control_gripper(self, gripper_position, duration=1.0, steps=10):
        """
        Control the gripper with velocity control by sending multiple individual commands
        over a specified duration.
        
        Args:
            gripper_position: Target gripper position (0.0 to 1.0)
            duration: Duration over which to move the gripper (seconds)
            steps: Number of intermediate steps
        """
        # Get current joint positions
        if not hasattr(self, '_last_joint_position'):
            self._last_joint_position = self.get_home_position()
        
        current_position = self._last_joint_position[:4]
        current_gripper = self._last_joint_position[4]
        
        step_time = duration / steps
        
        # Send multiple individual commands instead of one trajectory
        for i in range(steps + 1):
            proportion = i / steps
            intermediate_gripper = current_gripper + proportion * (gripper_position - current_gripper)
            
            # Send individual command for this step
            self.move_to_joint_position(current_position + [intermediate_gripper])
            self.get_logger().info(f"Gripper step {i+1}/{steps+1}: {intermediate_gripper:.2f}")
            
            # Wait a short time between steps
            time.sleep(step_time)
        
        # Update last position
        self._last_joint_position = current_position + [gripper_position]

    def berry_pick_and_place(self):
        """
        Pick a berry and place it in a cup without squeezing it.
        Uses gentle gripper control to avoid damaging the berry.
        """
        # Define locations
        berry_loc = {"x": -65, "y": 200, "z": 40}  # Berry on table
        cup_loc = {"x": 60, "y": 200, "z": 170}    # Cup 10cm to the right
    
        # Approach height above objects
        approach_height_berry = 100
        approach_height_cup = 40        
        # Save initial joint positions to use as _last_joint_position
        self._last_joint_position = self.get_home_position()
        
        self.get_logger().info("Starting berry pick and place sequence")
        
        # Move to home position
        self.get_logger().info("Moving to home position")
        self.move_to_joint_position(self.get_home_position())
        time.sleep(1)
        
        # BERRY PICKING SEQUENCE
        
        # Move above berry with gripper fully open
        self.get_logger().info(f"Moving above berry: ({berry_loc['x']}, {berry_loc['y']}, {berry_loc['z'] + approach_height_berry})")
        ik_result = inverse_kinematics(berry_loc["x"], berry_loc["y"], berry_loc["z"] + approach_height_berry)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach above berry: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper fully open
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles  # Update last position
        time.sleep(0.5)
        
        # Lower to grasp berry
        self.get_logger().info(f"Lowering to grasp berry: ({berry_loc['x']}, {berry_loc['y']}, {berry_loc['z']})")
        ik_result = inverse_kinematics(berry_loc["x"], berry_loc["y"], berry_loc["z"])
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach berry: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper still open
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles  # Update last position
        time.sleep(0.5)
        
        # Gently close gripper with velocity control (slower motion)
        self.get_logger().info("Gently closing gripper on berry")
        # Using 0.4 for partial closing (not full force) to avoid crushing
        self.control_gripper(0.7, duration=3.0, steps=30)  # Very slow and gentle closing
        time.sleep(0.5)
        
        # Lift berry carefully
        self.get_logger().info(f"Carefully lifting berry")
        ik_result = inverse_kinematics(berry_loc["x"], berry_loc["y"], berry_loc["z"] + approach_height_berry)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot lift berry: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.7  # Keep gripper partially closed
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles  # Update last position
        time.sleep(0.5)
        
        # Move to intermediate position (higher arc to avoid obstacles)
        self.get_logger().info("Moving through arc path to cup")
        ik_result = inverse_kinematics(0, 150, berry_loc["z"] + 100)  # Higher arc midpoint
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach midpoint: {ik_result}")
        else:
            _, theta1, theta2, theta3, theta4 = ik_result
            joint_angles = [
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.7  # Keep gripper partially closed
            ]
            self.move_to_joint_position(joint_angles)
            self._last_joint_position = joint_angles  # Update last position
            time.sleep(0.5)
        
        # Move above cup
        self.get_logger().info(f"Moving above cup: ({cup_loc['x']}, {cup_loc['y']}, {cup_loc['z'] + approach_height_cup})")
        ik_result = inverse_kinematics(cup_loc["x"], cup_loc["y"], cup_loc["z"] + approach_height_cup)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach above cup: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.7  # Keep holding berry
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles  # Update last position
        time.sleep(1.0)  # Longer pause for stability
        
        # Lower berry into cup
        self.get_logger().info(f"Lowering berry into cup: ({cup_loc['x']}, {cup_loc['y']}, {cup_loc['z']})")
        ik_result = inverse_kinematics(cup_loc["x"], cup_loc["y"], cup_loc["z"])
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach cup position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.7  # Still holding berry
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles  # Update last position
        time.sleep(1.0)  # Longer pause for stability
        
        # Gently open gripper to release berry
        self.get_logger().info("Gently releasing berry")
        self.control_gripper(0.0, duration=2.0, steps=20)  # Slow release
        time.sleep(0.5)
        
        # Raise arm out of cup carefully
        self.get_logger().info(f"Lifting from cup")
        ik_result = inverse_kinematics(cup_loc["x"], cup_loc["y"], cup_loc["z"] + approach_height_cup)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach lift position: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper fully open
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles  # Update last position
        time.sleep(1.0)
        
        # Return to home position
        self.get_logger().info("Berry successfully placed in cup! Returning to home position.")
        self.move_to_joint_position(self.get_home_position())
        time.sleep(1)

    def wipe_spill_sequence(self):
        """
        Picks up a sponge and wipes away a tea spill.
        
        1. Pick up sponge from location A
        2. Move to spill location B
        3. Perform thorough wiping motion to clean the spill
        4. Return to home position
        
        Uses a wide zig-zag pattern to cover a large area.
        """
        # Define locations
        sponge_loc = {"x": -80, "y": 180, "z": 50}  # Sponge location
        spill_loc = {"x": 20, "y": 180, "z": 60}     # Center of spill area
        
        # Spill size parameters (mm)
        spill_width = 80   # Width of spill area
        spill_height = 70    # Height of spill area
        
        # Approach height for safety
        approach_height = 70
        
        # Wiping pressure (how low to go during wiping)
        wipe_height = 40  # At surface level
        
        # Initialize tracking for joint positions
        self._last_joint_position = self.get_home_position()
        
        self.get_logger().info("Starting tea spill wiping sequence")
        
        # Move to home position
        self.get_logger().info("Moving to home position")
        self.move_to_joint_position(self.get_home_position())
        time.sleep(1)
        
        # SPONGE PICKUP SEQUENCE
        
        # Move above sponge with gripper open
        self.get_logger().info(f"Moving above sponge at ({sponge_loc['x']}, {sponge_loc['y']}, {sponge_loc['z'] + approach_height})")
        ik_result = inverse_kinematics(sponge_loc["x"], sponge_loc["y"], sponge_loc["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach above sponge: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.0  # Gripper fully open
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles
        time.sleep(0.5)
        
        # Lower to grasp sponge (SLOW DESCENT)
        self.get_logger().info(f"Slowly lowering to grasp sponge")
        
        # Calculate start and end positions
        start_z = sponge_loc["z"] + approach_height
        end_z = sponge_loc["z"]
        
        # Create multiple intermediate points for slow descent
        steps = 10  # Number of intermediate steps
        
        for i in range(steps + 1):
            # Calculate intermediate height
            progress = i / steps
            current_z = start_z - progress * (start_z - end_z)
            
            # Move to intermediate position
            self.get_logger().info(f"Descent step {i+1}/{steps+1}: height = {current_z:.1f}mm")
            ik_result = inverse_kinematics(sponge_loc["x"], sponge_loc["y"], current_z)
            if isinstance(ik_result, str):
                self.get_logger().error(f"Cannot reach intermediate position: {ik_result}")
                return
                
            _, theta1, theta2, theta3, theta4 = ik_result
            joint_angles = [
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.0  # Gripper open
            ]
            self.move_to_joint_position(joint_angles)
            self._last_joint_position = joint_angles
            time.sleep(0.2)  # Short delay between steps
        
        # Close gripper on sponge - firm but not crushing
        self.get_logger().info("Grasping sponge")
        self.control_gripper(0.7, duration=1.0, steps=10)  # Firm grip
        time.sleep(0.5)
        
        # Lift sponge
        self.get_logger().info(f"Lifting sponge")
        ik_result = inverse_kinematics(sponge_loc["x"], sponge_loc["y"], sponge_loc["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot lift sponge: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.7  # Maintain grip
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles
        time.sleep(0.5)
        
        # APPROACH SPILL AREA
        
        # Move to position above spill area
        self.get_logger().info(f"Moving above spill area")
        ik_result = inverse_kinematics(spill_loc["x"], spill_loc["y"], spill_loc["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach above spill: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.7  # Maintain grip on sponge
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles
        time.sleep(0.5)
        
        # WIPING SEQUENCE - ZIG-ZAG PATTERN
        
        # Calculate corner points of spill area
        left_edge = spill_loc["x"] - (spill_width / 2)
        right_edge = spill_loc["x"] + (spill_width / 2)
        
        # We'll do 5 passes for thorough cleaning
        passes = 5
        # Calculate spacing in Y-direction instead of Z
        y_spacing = spill_height / passes
        
        self.get_logger().info("Beginning zig-zag wiping pattern")
        
        # First lower to surface - this sets our constant Z height for wiping
        self.get_logger().info("Lowering sponge to surface")
        ik_result = inverse_kinematics(spill_loc["x"], spill_loc["y"], wipe_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot reach spill surface: {ik_result}")
            return
            
        _, theta1, theta2, theta3, theta4 = ik_result
        joint_angles = [
            np.radians(theta1), 
            np.radians(theta2), 
            np.radians(theta3), 
            np.radians(theta4), 
            0.7  # Maintain grip
        ]
        self.move_to_joint_position(joint_angles)
        self._last_joint_position = joint_angles
        time.sleep(0.5)
        
        # Perform zig-zag wiping motion with constant Z height
        for i in range(passes):
            # Calculate Y position for this pass
            current_y = spill_loc["y"] - (spill_height/2) + i * y_spacing
            
            # Move to left edge at current row with CONSTANT Z height
            self.get_logger().info(f"Wiping pass {i+1}/{passes}: moving to left edge")
            ik_result = inverse_kinematics(left_edge, current_y, wipe_height)
            if isinstance(ik_result, str):
                self.get_logger().warn(f"Cannot reach left edge at row {i+1}: {ik_result}")
                continue
                
            _, theta1, theta2, theta3, theta4 = ik_result
            joint_angles = [
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.7  # Maintain grip
            ]
            self.move_to_joint_position(joint_angles)
            self._last_joint_position = joint_angles
            time.sleep(0.2)
            
            # Wipe to right edge (slow and deliberate) with CONSTANT Z height
            self.get_logger().info(f"Wiping pass {i+1}/{passes}: wiping to right edge")
            ik_result = inverse_kinematics(right_edge, current_y, wipe_height)
            if isinstance(ik_result, str):
                self.get_logger().warn(f"Cannot reach right edge at row {i+1}: {ik_result}")
                continue
                
            _, theta1, theta2, theta3, theta4 = ik_result
            joint_angles = [
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.7  # Maintain grip
            ]
            self.move_to_joint_position(joint_angles)
            self._last_joint_position = joint_angles
            time.sleep(0.2)
            
            # Move to next row if not the last pass
            if i < passes - 1:
                next_y = spill_loc["y"] - (spill_height/2) + (i+1) * y_spacing
                
                # Move to start of next row with CONSTANT Z height
                self.get_logger().info(f"Moving to next row")
                ik_result = inverse_kinematics(right_edge, next_y, wipe_height)
                if isinstance(ik_result, str):
                    self.get_logger().warn(f"Cannot reach start of next row: {ik_result}")
                    continue
                    
                _, theta1, theta2, theta3, theta4 = ik_result
                joint_angles = [
                    np.radians(theta1), 
                    np.radians(theta2), 
                    np.radians(theta3), 
                    np.radians(theta4), 
                    0.7  # Maintain grip
                ]
                self.move_to_joint_position(joint_angles)
                self._last_joint_position = joint_angles
                time.sleep(0.2)
        
        # SPIRAL WIPING FOR FINAL POLISH - with constant Z height
        self.get_logger().info("Performing spiral pattern for final polish")
        
        # Generate spiral points - move center closer to robot for better reachability
        spiral_turns = 2
        points_per_turn = 12
        total_points = spiral_turns * points_per_turn
        
        # Adjust spiral center to be closer to robot
        spiral_center_x = spill_loc["x"] - 10  # Shifted slightly toward robot
        spiral_center_y = spill_loc["y"] - 20  # Closer to robot
        
        # Reduce spiral size for better reachability
        max_radius = min(spill_width, spill_height) * 0.4
        
        for i in range(total_points):
            # Calculate spiral position in XY plane only (constant Z)
            angle = (i / points_per_turn) * 2 * np.pi
            radius = (spiral_turns - (i / total_points) * spiral_turns) * max_radius
            
            x = spiral_center_x + radius * np.cos(angle)
            y = spiral_center_y + radius * np.sin(angle)  # Vary Y instead of Z
            
            # Move to spiral point with CONSTANT Z height
            ik_result = inverse_kinematics(x, y, wipe_height)
            if isinstance(ik_result, str):
                self.get_logger().warn(f"Cannot reach spiral point: {ik_result}")
                continue
                
            _, theta1, theta2, theta3, theta4 = ik_result
            joint_angles = [
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.7  # Maintain grip
            ]
            self.move_to_joint_position(joint_angles)
            self._last_joint_position = joint_angles
            time.sleep(0.1)  # Faster for spiral
        
        # COMPLETION SEQUENCE
        
        # Lift sponge from surface
        self.get_logger().info("Lifting sponge from surface")
        ik_result = inverse_kinematics(spill_loc["x"], spill_loc["y"], spill_loc["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot lift from surface: {ik_result}")
        else:
            _, theta1, theta2, theta3, theta4 = ik_result
            joint_angles = [
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.7  # Maintain grip
            ]
            self.move_to_joint_position(joint_angles)
            self._last_joint_position = joint_angles
            time.sleep(0.5)
        
        # Return to sponge location
        self.get_logger().info("Returning to sponge location")
        ik_result = inverse_kinematics(sponge_loc["x"], sponge_loc["y"], sponge_loc["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot return to sponge location: {ik_result}")
        else:
            _, theta1, theta2, theta3, theta4 = ik_result
            joint_angles = [
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.7  # Still holding sponge
            ]
            self.move_to_joint_position(joint_angles)
            self._last_joint_position = joint_angles
            time.sleep(0.5)
        
        # Lower sponge
        self.get_logger().info("Lowering sponge back to original position")
        ik_result = inverse_kinematics(sponge_loc["x"], sponge_loc["y"], sponge_loc["z"])
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot lower sponge: {ik_result}")
        else:
            _, theta1, theta2, theta3, theta4 = ik_result
            joint_angles = [
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.7  # Still holding sponge
            ]
            self.move_to_joint_position(joint_angles)
            self._last_joint_position = joint_angles
            time.sleep(0.5)
        
        # Release sponge
        self.get_logger().info("Releasing sponge")
        self.control_gripper(0.0, duration=1.0, steps=10)
        time.sleep(0.5)
        
        # Lift from sponge
        self.get_logger().info("Lifting from sponge")
        ik_result = inverse_kinematics(sponge_loc["x"], sponge_loc["y"], sponge_loc["z"] + approach_height)
        if isinstance(ik_result, str):
            self.get_logger().error(f"Cannot lift from sponge: {ik_result}")
        else:
            _, theta1, theta2, theta3, theta4 = ik_result
            joint_angles = [
                np.radians(theta1), 
                np.radians(theta2), 
                np.radians(theta3), 
                np.radians(theta4), 
                0.0  # Gripper open
            ]
            self.move_to_joint_position(joint_angles)
            self._last_joint_position = joint_angles
            time.sleep(0.5)
        
        # Return to home position
        self.get_logger().info("Wiping sequence complete! Returning to home position.")
        self.move_to_joint_position(self.get_home_position())
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller_4_1()

    # # First move to home position for safety
    # controller.move_to_joint_position(controller.get_home_position())
    # time.sleep(2)
    # # while True:
    # controller.execute_circle_trajectory(0, 130, 120, 40, num_points=200, delay=0.05, gripper_pos=0.8)
    # time.sleep(2)
    # controller.move_to_joint_position(controller.get_home_position())

    # controller.pick_and_place_sequence()
    # controller.berry_pick_and_place()
    controller.wipe_spill_sequence()
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
