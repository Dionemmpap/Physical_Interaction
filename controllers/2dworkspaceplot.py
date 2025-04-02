import numpy as np
import matplotlib.pyplot as plt

# Define Lynxmotion AL5A Link Lengths (in mm)
L1 = 50 # Base height
L2 = 105   # Shoulder to elbow
L3 = 105  # Elbow to wrist
L4 = 55  # Wrist to end-effector

# Updated Joint Limits (in radians)
theta1_range = np.radians([-90, 90])    # Base rotation (not used in 2D)
theta2_range = np.radians([45, 180])    # Shoulder: 80° to 170°
theta3_range = np.radians([0, 160])     # Elbow: 0° to 160°
theta4_range = np.radians([-45, 45])    # Wrist: -45° to 45°

# # No Limits (in radians)
# theta1_range = np.radians([-180, 180])    # Base rotation (not used in 2D)
# theta2_range = np.radians([-180, 180])    # Shoulder: 80° to 170°
# theta3_range = np.radians([-180, 180])     # Elbow: 0° to 160°
# theta4_range = np.radians([-180, 180])    # Wrist: -45° to 45°

# Base obstruction height (70 mm, approximated)
base_height = 50

# Function to compute forward kinematics in 2D (XZ-plane)
def forward_kinematics_2D(theta2, theta3, theta4):
    x0, z0 = 0, 0  # Base position
    x1, z1 = x0, z0 + L1  # Shoulder position
    x2 = x1 + L2 * np.cos(theta2)
    z2 = z1 + L2 * np.sin(theta2)
    x3 = x2 + L3 * np.cos(theta2 + theta3)
    z3 = z2 + L3 * np.sin(theta2 + theta3)
    x4 = x3 + L4 * np.cos(theta2 + theta3 + theta4)
    z4 = z3 + L4 * np.sin(theta2 + theta3 + theta4)

    # Apply ground constraint (Z ≥ 0)
    z4 = max(z4, 0)

    # Apply base obstruction constraint (cannot be at x=0 when z ≤ base height)
    if abs(x4) < 1e-6 and z4 <= base_height:
        x4 += 1  # Shift slightly to avoid singularity
    
    return [(x0, z0), (x1, z1), (x2, z2), (x3, z3), (x4, z4)]

# Plot the arm at a given joint configuration
def plot_robot_arm(theta2, theta3, theta4, ax, color='blue', label=''):
    points = forward_kinematics_2D(theta2, theta3, theta4)
    x_vals, z_vals = zip(*points)
    
    ax.plot(x_vals, z_vals, 'o-', color=color, markersize=5, label=label)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Z (mm)")
    ax.set_title("2D Robot Arm with Ground & Base Constraint")
    ax.set_xlim([-250, 250])
    ax.set_ylim([0, 400])
    
    # Plot ground level
    ax.axhline(y=0, color='black', linestyle='--', linewidth=2, label="Ground Level")  

    # Plot base obstruction (region where x=0, z ≤ base height)
    ax.fill_betweenx([0, base_height], -10, 10, color='gray', alpha=0.5, label="Base Obstruction")
    
    ax.legend()

# Generate workspace boundary points
def generate_workspace_boundary(samples=50):
    outer_boundary = []
    inner_boundary = []
    
    theta2_vals = np.linspace(theta2_range[0], theta2_range[1], samples)
    theta3_vals = np.linspace(theta3_range[0], theta3_range[1], samples)
    theta4_vals = np.linspace(theta4_range[0], theta4_range[1], samples)
    
    for theta2 in theta2_vals:
        for theta3 in theta3_vals:
            for theta4 in [theta4_range[0], theta4_range[1]]:  # Only max/min wrist
                x, z = forward_kinematics_2D(theta2, theta3, theta4)[-1]  # End-effector position
                
                # Apply base obstruction constraint
                if not (abs(x) < 1e-6 and z <= base_height):
                    outer_boundary.append((x, z))
    
    for theta2 in [theta2_range[0], theta2_range[1]]:  # Only max/min shoulder
        for theta3 in [theta3_range[0], theta3_range[1]]:  # Only max/min elbow
            for theta4 in theta4_vals:
                x, z = forward_kinematics_2D(theta2, theta3, theta4)[-1]  # End-effector position
                
                # Apply base obstruction constraint
                if not (abs(x) < 1e-6 and z <= base_height):
                    inner_boundary.append((x, z))
    
    return np.array(outer_boundary), np.array(inner_boundary)

# Plot the workspace boundaries
fig, ax = plt.subplots(figsize=(8, 6))

# Plot the arm at a default pose
plot_robot_arm(np.radians(45), np.radians(160), np.radians(0), ax, color='blue', label="Robot Arm")

# Generate and plot workspace boundaries
outer_boundary, inner_boundary = generate_workspace_boundary()
ax.scatter(outer_boundary[:, 0], outer_boundary[:, 1], color='red', s=2, label="Outer Boundary")
ax.scatter(inner_boundary[:, 0], inner_boundary[:, 1], color='green', s=2, label="Inner Boundary")

plt.legend()
plt.grid()
plt.show()
