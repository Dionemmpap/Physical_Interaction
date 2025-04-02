import numpy as np
import matplotlib.pyplot as plt
from controllers.workspacedifferentapproach3d import forward_kinematics_3D
from controllers.IK import forward_kinematics


# Define angle limits according to robot coordinates:
theta1_range = [-100, 90]  # rotating base
theta2_range = [-90, 45]   # shoulder to elbow
theta3_range = [-90, 85]   # elbow to wrist
theta4_range = [-95, 95]   # wrist to end-effector

# Define Lynxmotion AL5A Link Lengths (in mm)
L1 = 67   # Base height
L2 = 94   # Shoulder to elbow
L3 = 106  # Elbow to wrist
L4 = 85   # Wrist to end-effector

def complex_step_jacobian(theta_rad, h=1e-20):
    """
    Computes the Jacobian matrix using Complex Step Differentiation.
    
    Parameters:
        theta_rad (array): Joint angles in radians, in FK convention
        h (float): Small step size for complex differentiation

    Returns:
        J (ndarray): 3x4 Jacobian matrix
    """
    n = len(theta_rad)
    J = np.zeros((3, n))

    for i in range(n):
        theta_perturbed = np.array(theta_rad, dtype=np.complex128)
        theta_perturbed[i] += 1j * h

        # Use forward_kinematics_3D which expects radians in FK convention
        pos = forward_kinematics_3D(*theta_perturbed)[-1]
        J[:, i] = np.imag(pos) / h

    return J

def compute_trajectory(initial_angles_deg, v_desired, delta_t=0.1, num_steps=100):
    """
    Computes a trajectory based on desired end-effector velocity.
    
    Args:
        initial_angles_deg: Joint angles in robot convention (degrees)
        v_desired: Desired end-effector velocity [vx, vy, vz] in mm/s
        delta_t: Time step in seconds
        num_steps: Maximum number of steps to compute
        
    Returns:
        time_steps: List of time points
        robot_angles: List of joint angles at each time point (in robot convention, degrees)
        positions: Array of end-effector positions at each time point
        joint_limits_hit: List of which joints hit limits at each time step
    """
    # Convert velocity to numpy array if it's not already
    v_desired = np.array(v_desired)
    
    # Initialize output lists
    time_steps = [0.0]
    robot_angles = [initial_angles_deg.copy()]
    
    # Calculate initial position using IK's forward_kinematics function
    initial_position = forward_kinematics(*initial_angles_deg)[-1]
    positions = [initial_position]
    
    # Current angles in robot convention (degrees)
    theta_current_robot_deg = np.array(initial_angles_deg)
    
    # Track which joints hit limits at each step
    joint_limits_hit = [[False, False, False, False]]
    
    # Debug output for initial angles
    print(f"Initial robot angles (deg): {theta_current_robot_deg}")
    print(f"Initial position: {initial_position}")
    
    # Execute motion
    for step in range(num_steps):
        # Convert current robot angles to FK convention for Jacobian calculation
        theta1_FK = theta_current_robot_deg[0]
        theta2_FK = theta_current_robot_deg[1] + 90
        theta3_FK = -(theta_current_robot_deg[2] + 90)
        theta4_FK = theta_current_robot_deg[3]
        
        # Convert to radians for Jacobian calculation
        theta_FK_rad = np.radians([theta1_FK, theta2_FK, theta3_FK, theta4_FK])
        
        # Calculate Jacobian
        J = complex_step_jacobian(theta_FK_rad)
        
        # Check for singularity
        cond_num = np.linalg.cond(J)
        if cond_num > 1e4:
            print(f"Warning: Near singularity at step {step}, time {time_steps[-1]:.2f}s")
            print(f"Condition number: {cond_num:.2f}")
            break
        
        # Calculate joint velocities
        q_dot = np.linalg.pinv(J) @ v_desired
        
        # Check which joints would exceed limits in the next step
        joint_at_limit = [False, False, False, False]
        
        # Convert predicted angles to degrees for limit checking
        next_FK_rad = theta_FK_rad + q_dot * delta_t
        next_FK_deg = np.degrees(next_FK_rad)
        
        # Convert to robot convention
        next_robot_deg = [
            next_FK_deg[0],  # theta1
            next_FK_deg[1] - 90,  # theta2
            -(next_FK_deg[2] + 90),  # theta3
            next_FK_deg[3]  # theta4
        ]
        
        # Check limits and zero out velocities for joints at limits
        limits = [theta1_range, theta2_range, theta3_range, theta4_range]
        for i in range(4):
            if next_robot_deg[i] < limits[i][0]:  # Below lower limit
                joint_at_limit[i] = True
                # Clamp the value at the lower limit
                next_robot_deg[i] = limits[i][0]
                print(f"Joint {i+1} hit lower limit ({limits[i][0]}°) at step {step}")
                q_dot[i] = 0  # Zero velocity for this joint
                
            elif next_robot_deg[i] > limits[i][1]:  # Above upper limit
                joint_at_limit[i] = True
                # Clamp the value at the upper limit
                next_robot_deg[i] = limits[i][1]
                print(f"Joint {i+1} hit upper limit ({limits[i][1]}°) at step {step}")
                q_dot[i] = 0  # Zero velocity for this joint
        
        # Store which joints hit their limits
        joint_limits_hit.append(joint_at_limit.copy())
        
        # If we've hit limits, recalculate the FK angles to ensure consistency
        if any(joint_at_limit):
            # Convert back to FK convention
            next_FK_deg = [
                next_robot_deg[0],
                next_robot_deg[1] + 90,
                -next_robot_deg[2] - 90,
                next_robot_deg[3]
            ]
            next_FK_rad = np.radians(next_FK_deg)
        else:
            # If no limits hit, use the calculated next position
            next_FK_rad = theta_FK_rad + q_dot * delta_t
            
        # Update FK angles (in radians)
        theta_FK_rad = next_FK_rad
        
        # Convert back to degrees
        theta_FK_deg = np.degrees(theta_FK_rad)
        
        # Convert FK angles back to robot convention (degrees)
        theta_robot_deg = [
            theta_FK_deg[0],
            theta_FK_deg[1] - 90,
            -(theta_FK_deg[2] + 90),
            theta_FK_deg[3]
        ]
        
        # Update current robot angles
        theta_current_robot_deg = np.array(theta_robot_deg)
        
        # Store the new robot angles
        robot_angles.append(theta_robot_deg.copy())
        
        # Calculate new position using IK's forward_kinematics for consistency
        new_position = forward_kinematics(*theta_robot_deg)[-1]
        positions.append(new_position)
        
        # Update time
        time_steps.append(time_steps[-1] + delta_t)
    
    # Debug output for final state
    print(f"Final robot angles (deg): {robot_angles[-1]}")
    print(f"Final position: {positions[-1]}")
    
    return time_steps, robot_angles, np.array(positions)

# -------------------- Example Usage --------------------

if __name__ == "__main__":
    # Simulation Parameters
    theta_degrees = [0, 0, 0, 0]  # Initial joint angles in degrees
    v_desired = np.array([0, 5.0, 0.0])  # Desired end-effector velocity (mm/s)
    delta_t = 0.1  # Time step (s)
    num_steps = 100  # Number of iterations

    # Compute trajectory
    time_steps, robot_angles, trajectory, joint_limits_hit = compute_trajectory(
        theta_degrees, v_desired, delta_t, num_steps
    )
    
    # Print first few trajectory points
    print("\nTrajectory Points (first 5):")
    print("Time (s) | Robot Joint Angles (degrees)")
    print("-" * 50)
    
    for i in range(min(5, len(time_steps))):
        angles_str = ", ".join(f"{a:.4f}" for a in robot_angles[i])
        print(f"{time_steps[i]:.2f}s | [{angles_str}]")
    
    if len(time_steps) > 5:
        print("...")
    
    print(f"\nTrajectory completed with {len(time_steps)} points")
    print(f"Final position: {trajectory[-1]}")
    
    # -------------------- Visualization --------------------
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label="End-Effector Path")
    ax.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], color='red', label="Start")
    ax.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], color='green', label="End")
    
    ax.set_xlabel("X-axis (mm)")
    ax.set_ylabel("Y-axis (mm)")
    ax.set_zlabel("Z-axis (mm)")
    ax.set_title("End-Effector Trajectory")
    ax.legend()
    plt.show()
    
    # -------------------- Additional Analysis --------------------
    
    # Plot joint angles over time
    plt.figure(figsize=(10, 6))
    for i in range(4):
        plt.plot(time_steps, [angles[i] for angles in robot_angles], 
                 label=f"Joint {i+1}")
    
    plt.xlabel("Time (s)")
    plt.ylabel("Joint Angle (rad)")
    plt.title("Joint Angles vs. Time")
    plt.grid(True)
    plt.legend()
    plt.show()
