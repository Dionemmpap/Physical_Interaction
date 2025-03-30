import rclpy
import numpy as np
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class JointVerifier(Node):
    def __init__(self):
        super().__init__('joint_verifier')
        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        
        # Create parameters for controlling each joint
        self.declare_parameter('joint1', 0.0)
        self.declare_parameter('joint2', 0.0)
        self.declare_parameter('joint3', 0.0)
        self.declare_parameter('joint4', 0.0)
        self.declare_parameter('gripper', 0.0)
        
        # Create a timer for continuous monitoring
        self.timer = self.create_timer(0.1, self.update_joint_positions)
        
        # Store previous joint values to detect changes
        self.prev_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info("Joint Verifier started. Use parameters to move joints:")
        self.get_logger().info("ros2 param set /joint_verifier joint1 0.5")
        
    def update_joint_positions(self):
        # Get current parameter values
        joint_values = [
            self.get_parameter('joint1').get_parameter_value().double_value,
            self.get_parameter('joint2').get_parameter_value().double_value,
            self.get_parameter('joint3').get_parameter_value().double_value,
            self.get_parameter('joint4').get_parameter_value().double_value,
            self.get_parameter('gripper').get_parameter_value().double_value
        ]
        
        # Check if any joint value has changed
        if joint_values != self.prev_joint_values:
            self.get_logger().info(f"Moving to joint positions: {joint_values}")
            self.move_to_joint_position(joint_values)
            self.prev_joint_values = joint_values
    
    def move_to_joint_position(self, joint_positions):
        # Convert all elements to float
        joint_positions = [float(pos) for pos in joint_positions]

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "gripper"]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()

        msg.points = [point]
        self._publisher.publish(msg)
        
    def run_verification_sequence(self):
        """Run a sequence to verify each joint's movement"""
        self.get_logger().info("Starting joint verification sequence")
        
        # Start with all joints at 0
        self.move_to_joint_position([0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(2)
        
        # Move each joint individually
        for joint_idx in range(5):
            joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Move joint to positive position
            joint_positions[joint_idx] = 0.5  # 0.5 radian movement
            self.get_logger().info(f"Moving joint {joint_idx+1} to position 0.5")
            self.move_to_joint_position(joint_positions)
            time.sleep(2)
            
            # Return to zero
            joint_positions[joint_idx] = 0.0
            self.move_to_joint_position(joint_positions)
            time.sleep(1)
            
            # Move joint to negative position (except gripper)
            if joint_idx < 4:  # Don't do negative for gripper
                joint_positions[joint_idx] = -0.5
                self.get_logger().info(f"Moving joint {joint_idx+1} to position -0.5")
                self.move_to_joint_position(joint_positions)
                time.sleep(2)
                
                # Return to zero
                joint_positions[joint_idx] = 0.0
                self.move_to_joint_position(joint_positions)
                time.sleep(1)
        
        self.get_logger().info("Verification sequence complete")


def main(args=None):
    rclpy.init(args=args)
    verifier = JointVerifier()
    
    # Run the automatic verification sequence
    verifier.run_verification_sequence()
    
    # Then continue running for manual parameter control
    rclpy.spin(verifier)
    
    verifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()