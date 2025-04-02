import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from scipy.spatial import ConvexHull
import plotly.io as pio
pio.renderers.default = "browser"

# Define robot link lengths (in mm)
shoulder_to_elbow = 50
elbow_to_wrist = 105
wrist_to_gripper = 105
base_height = 55

# Define realistic joint limits based on Lynxmotion AL5A specs
joint_limits = [
    (-np.pi/2, np.pi/2),  # Base: -90° to 90°
    (np.radians(45), np.radians(160)),  # Shoulder: 45° to 160°
    (0, np.radians(160)),  # Elbow: 0° to 160°
    (np.radians(-90), np.radians(90))  # Wrist: -90° to 90°
]

# Forward Kinematics function
def forward_kinematics(theta1, theta2, theta3, theta4):
    x = np.cos(theta1) * (
        shoulder_to_elbow * np.cos(theta2) +
        elbow_to_wrist * np.cos(theta2 + theta3) +
        wrist_to_gripper * np.cos(theta2 + theta3 + theta4)
    )
    y = np.sin(theta1) * (
        shoulder_to_elbow * np.cos(theta2) +
        elbow_to_wrist * np.cos(theta2 + theta3) +
        wrist_to_gripper * np.cos(theta2 + theta3 + theta4)
    )
    z = base_height + (
        shoulder_to_elbow * np.sin(theta2) +
        elbow_to_wrist * np.sin(theta2 + theta3) +
        wrist_to_gripper * np.sin(theta2 + theta3 + theta4)
    )
    return [x, y, max(z, 0)]  # Ensure ground constraint (Z >= 0)

# Generate workspace points
def generate_workspace(joint_constraints=True, samples=30):
    if joint_constraints:
        theta_range = [np.linspace(joint_limits[i][0], joint_limits[i][1], samples) for i in range(4)]
    else:
        theta_range = [np.linspace(-np.pi, np.pi, samples) for _ in range(4)]
    
    points = []
    for t1 in theta_range[0]:
        for t2 in theta_range[1]:
            for t3 in theta_range[2]:
                for t4 in theta_range[3]:
                    points.append(forward_kinematics(t1, t2, t3, t4))
    
    return np.array(points)

# Plot workspace with improved visualization
def plot_workspace(workspace_points, title, cross_section=False):
    hull = ConvexHull(workspace_points)
    
    # Color mapping based on Z-axis
    z_values = workspace_points[:, 2]
    colors = z_values - np.min(z_values)
    colors = colors / np.max(colors)  # Normalize
    
    fig = go.Figure()
    
    # 3D Surface
    fig.add_trace(go.Mesh3d(
        x=workspace_points[:, 0], 
        y=workspace_points[:, 1], 
        z=workspace_points[:, 2],
        colorbar_title='Height',
        colorscale='Viridis',
        opacity=0.5,
        i=hull.simplices[:, 0],
        j=hull.simplices[:, 1],
        k=hull.simplices[:, 2]
    ))
    
    # Cross-section at Y=0
    if cross_section:
        cross_section_points = workspace_points[np.abs(workspace_points[:, 1]) < 0.1]
        fig.add_trace(go.Scatter3d(
            x=cross_section_points[:, 0],
            y=cross_section_points[:, 1],
            z=cross_section_points[:, 2],
            mode='markers',
            marker=dict(size=2, color='red'),
            name='Cross-section'
        ))
    
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'
        )
    )
    
    fig.show()

# Generate and plot constrained workspace
workspace_constrained = generate_workspace(joint_constraints=True)
plot_workspace(workspace_constrained, 'Constrained 3D Workspace', cross_section=True)

# Generate and plot unconstrained workspace
workspace_unconstrained = generate_workspace(joint_constraints=False)
plot_workspace(workspace_unconstrained, 'Unconstrained 3D Workspace', cross_section=True)
