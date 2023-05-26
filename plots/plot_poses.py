import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, art3d
import numpy as np

poses_xy = [
    [0.132, 0.265, 0.440],
    [0.093, 0.305, 0.214],
    [0.273, 0.374, 0.214],
    [0.273, 0.225, 0.214],
    [-0.077, 0.225, 0.214],
    [-0.077, 0.354, 0.214]
]

poses_z = [
    [0.132, 0.265, 0.440],
    [0.094, 0.304, 0.078],
    [0.094, 0.304, 0.278],
    [0.144, 0.254, 0.478],
    [0.164, 0.254, 0.628]
]

# Extract x, y, z coordinates for xy poses
poses_xy = np.array(poses_xy)
x_xy = poses_xy[:, 0]
y_xy = poses_xy[:, 1]
z_xy = poses_xy[:, 2]

# Extract x, y, z coordinates for z poses
poses_z = np.array(poses_z)
x_z = poses_z[:, 0]
y_z = poses_z[:, 1]
z_z = poses_z[:, 2]

# Calculate the vectors pointing from each xy pose to the next
vectors_x_xy = np.diff(x_xy)
vectors_y_xy = np.diff(y_xy)
vectors_z_xy = np.diff(z_xy)

# Calculate the vectors pointing from each z pose to the next
vectors_x_z = np.diff(x_z)
vectors_y_z = np.diff(y_z)
vectors_z_z = np.diff(z_z)

# Plot for xy poses
fig_xy = plt.figure()
ax_xy = fig_xy.add_subplot(111, projection='3d')
ax_xy.quiver(x_xy[:-1], y_xy[:-1], z_xy[:-1], vectors_x_xy, vectors_y_xy, vectors_z_xy,
             color='blue', label='Translation XY', arrow_length_ratio=0.2)  # Adjust arrow size here
ax_xy.scatter(0, 0, 0, color='blue', label='base_link')
ax_xy.text(0, 0, 0, f'base_link', color='black', fontsize=8)
ax_xy.scatter(x_xy[0], y_xy[0], z_xy[0], color='red', label='Poses')  # Plot pose 1 separately
ax_xy.scatter(x_xy[1:], y_xy[1:], z_xy[1:], color='red')  # Plot other poses
ax_xy.set_xlabel('X')
ax_xy.set_ylabel('Y')
ax_xy.set_zlabel('Z')
ax_xy.set_title('Translation XY\nfrom base_link')
ax_xy.legend()

# Set plot limits for better visualization
ax_xy.set_xlim([-0.1, 0.4])
ax_xy.set_ylim([0, 0.5])
ax_xy.set_zlim([0, 0.5])

# Plot for z poses
fig_z = plt.figure()
ax_z = fig_z.add_subplot(111, projection='3d')
ax_z.quiver(x_z[:-1], y_z[:-1], z_z[:-1], vectors_x_z, vectors_y_z, vectors_z_z,
            color='blue', label='Translation Z', arrow_length_ratio=0.2)  # Adjust arrow size here
ax_z.scatter(0, 0, 0, color='blue', label='base_link')
ax_z.text(0, 0, 0, f'base_link', color='black', fontsize=8)
ax_z.scatter(x_z[0], y_z[0], z_z[0], color='red', label='Poses')  # Plot pose 1 separately
ax_z.scatter(x_z[1:], y_z[1:], z_z[1:], color='red')  # Plot other poses
ax_z.set_xlabel('X')
ax_z.set_ylabel('Y')
ax_z.set_zlabel('Z')
ax_z.set_title('Translation Z\nfrom base_link')
ax_z.legend()

# Set plot limits for better visualization
ax_z.set_xlim([0, 0.4])
ax_z.set_ylim([0, 0.4])
ax_z.set_zlim([0, 1])

# Add labels to each pose point
for i, pose in enumerate(poses_xy):
    ax_xy.text(pose[0], pose[1], pose[2], f'Pose {i+1}', color='black', fontsize=8)

for i, pose in enumerate(poses_z):
    ax_z.text(pose[0], pose[1], pose[2], f'Pose {i+1}', color='black', fontsize=8)

# Show the plots
plt.show()

