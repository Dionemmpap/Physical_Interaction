import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



# Define Lynxmotion AL5A Link Lengths (in mm)
L1 = 67 # Base height
L2 = 94   # Shoulder to elbow
L3 = 106  # Elbow to wrist
L4 = 85  # Wrist to end-effector


def forward_kinematics(theta1, theta2, theta3, theta4):
    """ Compute joint positions from joint angles """
    # Convert degrees to radians
    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2 + 90)  # Adjusted to match inverse kinematics convention
    theta3 = np.radians(-theta3 - 90)
    theta4 = np.radians(theta4)
    # Base position
    x0, y0, z0 = 0, 0, 0
    # Shoulder position
    x1, y1, z1 = 0, 0, L1
    # Elbow position
    x2 = L2 * np.cos(theta2) * np.cos(theta1)
    y2 = L2 * np.cos(theta2) * np.sin(theta1)
    z2 = L1 + L2 * np.sin(theta2)
    # Wrist position
    x3 = x2 + L3 * np.cos(theta2 + theta3) * np.cos(theta1)
    y3 = y2 + L3 * np.cos(theta2 + theta3) * np.sin(theta1)
    z3 = z2 + L3 * np.sin(theta2 + theta3)
    # End-effector position
    x4 = x3 + L4 * np.cos(theta2 + theta3 + theta4) * np.cos(theta1)
    y4 = y3 + L4 * np.cos(theta2 + theta3 + theta4) * np.sin(theta1)
    z4 = z3 + L4 * np.sin(theta2 + theta3 + theta4)
    return np.array([[x0, y0, z0], 
                     [x1, y1, z1], 
                     [x2, y2, z2], 
                     [x3, y3, z3], 
                         [x4, y4, z4]])





# inverse kinematics for set wrist end effector angle
def inverse_kinematics(x, y, z, theta_wrist=-90):
    if z < 0:
        return "Position out of reach"
    
    # Solve for theta_1
    theta1 = np.degrees(np.arctan2(y, x))  # Fix xy swap

    # Projection onto the xy-plane
    r = np.sqrt(x**2 + y**2)
    
    # Effective reach without base height (L1)
    z_prime = z - L1


    r_double_prime = r - L4 * np.cos(np.radians(theta_wrist))
    z_double_prime = z_prime - L4 * np.sin(np.radians(theta_wrist))

    # Effective reach without wrist length (L4)
    d = np.sqrt(r_double_prime**2 + z_double_prime**2)

    # Solve for theta_3 using the Law of Cosines
    cos_theta3 = (L2**2 + L3**2 - d**2) / (2 * L2 * L3)
    cos_theta3 = np.clip(cos_theta3, -1, 1)  # Prevent NaN

    theta3 = np.degrees(np.arccos(cos_theta3))

    # Solve for theta_2
    alpha = np.arctan2(z_double_prime, r_double_prime)
    cos_beta = (d**2 + L2**2 - L3**2) / (2 * d * L2)
    cos_beta = np.clip(cos_beta, -1, 1)  # Ensure it's in valid range
    beta = np.arccos(cos_beta)  # Now safe to compute

    beta = np.arccos((d**2 + L2**2 - L3**2) / (2 * d * L2))

    theta2_up = np.degrees(alpha + beta)
    theta3_up = -(180 - theta3)

    theta2_down = np.degrees(alpha - beta)
    theta3_down = 180 - theta3

    # Solve for theta_4
    theta4_up = theta_wrist - (theta2_up + theta3_up)
    theta4_down = theta_wrist - (theta2_down + theta3_down)

    # Define limits
    theta1_range = [-100, 90]
    theta2_range = [-90, 45]
    theta3_range = [-90, 85]
    theta4_range = [-95, 95]

    # Convert angles to robot coordinates
    theta2_robot_up = theta2_up - 90
    theta2_robot_down = theta2_down - 90
    theta3_robot_up = -(theta3_up + 90)
    theta3_robot_down = -(theta3_down + 90)

    # Check limits
    if (theta1_range[0] <= theta1 <= theta1_range[1] and
        theta2_range[0] <= theta2_robot_up <= theta2_range[1] and
        theta3_range[0] <= theta3_robot_up <= theta3_range[1] and
        theta4_range[0] <= theta4_up <= theta4_range[1]):
        return ["Elbow up:", theta1, theta2_robot_up, theta3_robot_up, theta4_up]
    elif (theta1_range[0] <= theta1 <= theta1_range[1] and
          theta2_range[0] <= theta2_robot_down <= theta2_range[1] and
          theta3_range[0] <= theta3_robot_down <= theta3_range[1] and
          theta4_range[0] <= theta4_down <= theta4_range[1]):
        return ["Elbow down:", theta1, theta2_robot_down, theta3_robot_down, theta4_down]
    else:
        return "Position out of reach"


# Test the given positions
s1 = [100.0, 100.0, 50.0, -45]
s2 = [200.0, 100.0, 300.0, 45]
s3 = [0.0, 0.0, 300.0, -45]
s4 = [0.0, 0.0, 70.0, -90]

print(inverse_kinematics(*s1))
print(inverse_kinematics(*s2))
print(inverse_kinematics(*s3))
print(inverse_kinematics(*s4))





def plot_robot_arm(joint_positions, solution_label):
    """ Plot the robotic arm in 3D space """
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    # Extract coordinates
    x = joint_positions[:, 0]
    y = joint_positions[:, 1]
    z = joint_positions[:, 2]
    # Plot arm links
    ax.plot(x, y, z, marker='o', markersize=6, label=solution_label, color="blue", linewidth=2)
    # Plot base
    ax.scatter([0], [0], [0], color="red", s=100, label="Base")
    # Labels and Limits
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("3D Simulation of Robot Arm")
    ax.set_xlim([-200, 200])
    ax.set_ylim([-200, 200])
    ax.set_zlim([0, 300])
    ax.legend()
    plt.show()
    
# Plot s1 (only feasible position) without elbow up string
plot_robot_arm(forward_kinematics(*inverse_kinematics(*s1)[1:]), "Elbow up")
