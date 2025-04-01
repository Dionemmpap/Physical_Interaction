import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import plotly.io as pio
# pio.renderers.default = "browser"
# Define Lynxmotion AL5A Link Lengths (in mm)
L1 = 52 # Base height
L2 = 82.5   # Shoulder to elbow
L3 = 86.0  # Elbow to wrist
L4 = 75.0  # Wrist to end-effector

# # Joint Limits (in radians)
# theta1_range = np.radians([-90, 90])    # Base rotation
# theta2_range = np.radians([45, 180])    # Shoulder
# theta3_range = np.radians([0, 160])     # Elbow
# theta4_range = np.radians([-90, 90])    # Wrist

#without limits 
theta1_range = np.radians([-180, 180])    # Base rotation
theta2_range = np.radians([0, 180])    # Shoulder
theta3_range = np.radians([-180, 180])     # Elbow
theta4_range = np.radians([-180, 180])    # Wrist

# Base obstruction height (mm)
base_height = 52  

# Forward Kinematics for 3D (X, Y, Z)
def forward_kinematics_3D(theta1, theta2, theta3, theta4):
    x0, y0, z0 = 0, 0, 0  # Base position
    x1, y1, z1 = 0, 0, L1  # Shoulder position

    x2 = x1 + L2 * np.cos(theta2)
    z2 = z1 + L2 * np.sin(theta2)
    y2 = 0  # Arm still in X-Z plane before base rotation

    x3 = x2 + L3 * np.cos(theta2 + theta3)
    z3 = z2 + L3 * np.sin(theta2 + theta3)
    y3 = 0

    x4 = x3 + L4 * np.cos(theta2 + theta3 + theta4)
    z4 = z3 + L4 * np.sin(theta2 + theta3 + theta4)
    y4 = 0

    # Apply base rotation (θ1) -> Rotate around Z-axis
    x1, y1 = x1 * np.cos(theta1), x1 * np.sin(theta1)
    x2, y2 = x2 * np.cos(theta1), x2 * np.sin(theta1)
    x3, y3 = x3 * np.cos(theta1), x3 * np.sin(theta1)
    x4, y4 = x4 * np.cos(theta1), x4 * np.sin(theta1)

    # Apply ground constraint (Z ≥ 0)
    z4 = max(z4, 0)

    # Apply base obstruction constraint (x, y close to 0 and z ≤ base height)
    if abs(x4) < 1e-6 and abs(y4) < 1e-6 and z4 <= base_height:
        x4 += 1  # Small shift to avoid singularity
    
    return [(x0, y0, z0), (x1, y1, z1), (x2, y2, z2), (x3, y3, z3), (x4, y4, z4)]

# 3D Plot of the Robot Arm
def plot_robot_arm(theta1, theta2, theta3, theta4, ax, color='blue', label=''):
    points = forward_kinematics_3D(theta1, theta2, theta3, theta4)
    x_vals, y_vals, z_vals = zip(*points)

    ax.plot(x_vals, y_vals, z_vals, 'o-', color=color, markersize=5, label=label)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("3D Robot Arm with Base Rotation")
    ax.set_xlim([-250, 250])
    ax.set_ylim([-250, 250])
    ax.set_zlim([0, 400])

    # Plot ground level
    ax.plot([-250, 250], [0, 0], [0, 0], 'k--', linewidth=2, label="Ground Level")  

    ax.legend()

# Generate 3D Workspace Boundaries
def generate_workspace_boundary(samples=25):
    workspace = []
    
    theta1_vals = np.linspace(theta1_range[0], theta1_range[1], samples)
    theta2_vals = np.linspace(theta2_range[0], theta2_range[1], samples)
    theta3_vals = np.linspace(theta3_range[0], theta3_range[1], samples)
    theta4_vals = np.linspace(theta4_range[0], theta4_range[1], samples)

    for theta1 in theta1_vals:
        for theta2 in theta2_vals:
            for theta3 in theta3_vals:
                for theta4 in theta4_vals:  # Only min/max wrist
                    x, y, z = forward_kinematics_3D(theta1, theta2, theta3, theta4)[-1]  # End-effector pos

                    # Apply base obstruction constraint
                    # if not (abs(x) < 50 and z <= base_height):
                        
                    if not (abs(x) < 50 and abs(y) < 50 and abs(z) < 52 ):
                        workspace.append((x, y, z))

    return np.array(workspace)

def main():
    # Plot in 3D
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot arm at a default pose (θ1=0°, θ2=90°, θ3=80°, θ4=0°)
    plot_robot_arm(np.radians(0), np.radians(45), np.radians(160), np.radians(0), ax, color='blue', label="Robot Arm")

    # Generate and plot workspace boundary
    workspace = generate_workspace_boundary()
    if workspace.size > 0:
        ax.scatter(workspace[:, 0], workspace[:, 1], workspace[:, 2], color='red', s=1, alpha=0.5, label="Workspace")

    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    main()

