import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import random

# Set up the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_zlim(0, 100)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Number of birds in the flock
num_birds = 10

# Initialize positions and directions for each bird
bird_positions = np.random.rand(num_birds, 3) * 100  # Random positions within the 100x100x100 space
bird_directions = np.random.rand(num_birds, 3) * 2 - 1  # Random initial directions between -1 and 1 for each axis

# Normalize the initial directions
for i in range(num_birds):
    norm = np.linalg.norm(bird_directions[i])
    bird_directions[i] /= norm

# Plot the birds (using different markers for visibility)
birds = [ax.plot([bird_positions[i, 0]], [bird_positions[i, 1]], [bird_positions[i, 2]],
                 marker=">", color='orange', markersize=10)[0] for i in range(num_birds)]

# Function to calculate speed based on distance to the boundary
def calculate_speed(position, boundary_min, boundary_max, min_speed=0.5):
    distance_to_min = position - boundary_min
    distance_to_max = boundary_max - position
    distance = min(distance_to_min, distance_to_max)
    
    # Adjust the divisor to slow down the bird's speed, ensure a minimum speed
    speed = max(min_speed, (distance / 20.0))
    return speed

# Update function for animation
def update(frame):
    global bird_positions, bird_directions

    # Update each bird's position and direction
    for i in range(num_birds):
        # Calculate speeds based on distance to boundaries, with increased minimum speed near boundaries
        speedX = calculate_speed(bird_positions[i, 0], 0, 100)
        speedY = calculate_speed(bird_positions[i, 1], 0, 100)
        speedZ = calculate_speed(bird_positions[i, 2], 0, 100)

        # Update the bird's position
        bird_positions[i, 0] += speedX * bird_directions[i, 0]
        bird_positions[i, 1] += speedY * bird_directions[i, 1]
        bird_positions[i, 2] += speedZ * bird_directions[i, 2]

        # Check for boundary conditions and forcefully adjust direction if needed
        if bird_positions[i, 0] >= 100 or bird_positions[i, 0] <= 0:
            bird_directions[i, 0] *= -1  # Reverse direction on X-axis
            bird_positions[i, 0] = np.clip(bird_positions[i, 0], 1, 99)  # Move away from boundary

        if bird_positions[i, 1] >= 100 or bird_positions[i, 1] <= 0:
            bird_directions[i, 1] *= -1  # Reverse direction on Y-axis
            bird_positions[i, 1] = np.clip(bird_positions[i, 1], 1, 99)  # Move away from boundary

        if bird_positions[i, 2] >= 100 or bird_positions[i, 2] <= 0:
            bird_directions[i, 2] *= -1  # Reverse direction on Z-axis
            bird_positions[i, 2] = np.clip(bird_positions[i, 2], 1, 99)  # Move away from boundary

        # Normalize the direction vector to maintain consistent movement
        norm = np.linalg.norm(bird_directions[i])
        bird_directions[i] /= norm

        # Update the bird's position in the plot
        birds[i].set_data([bird_positions[i, 0]], [bird_positions[i, 1]])
        birds[i].set_3d_properties([bird_positions[i, 2]])

    return birds

# Create the animation
ani = FuncAnimation(fig, update, frames=200, interval=20, blit=False)

# Show the plot
plt.show()
