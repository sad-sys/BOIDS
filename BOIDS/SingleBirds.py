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

# Bird initial position
birdX = 25
birdY = 25
birdZ = 25

# Parameters for bird movement
directionX = 1  # 1 for increasing X, -1 for decreasing X
directionY = 1  # 1 for increasing Y, -1 for decreasing Y
directionZ = 1  # 1 for increasing Z, -1 for decreasing Z

# Set the bird to look like an orange >
bird, = ax.plot([birdX], [birdY], [birdZ], marker=">", color='orange', markersize=10)

# Function to calculate speed based on distance to the boundary
def calculate_speed(position, boundary_min, boundary_max):
    distance_to_min = position - boundary_min
    distance_to_max = boundary_max - position
    distance = min(distance_to_min, distance_to_max)
    
    # Adjust the divisor to slow down the bird's speed
    speed = max(0.1, (distance / 20.0))
    return speed


# Function to randomly rotate direction vector slightly
def random_rotate_direction(dirX, dirY, dirZ):
    angle = random.uniform(-np.pi / 12, np.pi / 12)  # Random angle between -15 to 15 degrees
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    
    # Randomly choose rotation axis
    axis = random.choice(['X', 'Y', 'Z'])
    
    if axis == 'X':
        new_dirY = cos_angle * dirY - sin_angle * dirZ
        new_dirZ = sin_angle * dirY + cos_angle * dirZ
        return dirX, new_dirY, new_dirZ
    elif axis == 'Y':
        new_dirX = cos_angle * dirX + sin_angle * dirZ
        new_dirZ = -sin_angle * dirX + cos_angle * dirZ
        return new_dirX, dirY, new_dirZ
    else:
        new_dirX = cos_angle * dirX - sin_angle * dirY
        new_dirY = sin_angle * dirX + cos_angle * dirY
        return new_dirX, new_dirY, dirZ

# Update function for animation
def update(frame):
    global birdX, birdY, birdZ, directionX, directionY, directionZ

    # Calculate speeds based on distance to boundaries
    speedX = calculate_speed(birdX, 0, 100)
    speedY = calculate_speed(birdY, 0, 100)
    speedZ = calculate_speed(birdZ, 0, 100)

    # Update the bird's position
    birdX += speedX * directionX
    birdY += speedY * directionY
    birdZ += speedZ * directionZ

    # Check for boundary conditions and reverse direction if needed
    if birdX >= 100 or birdX <= 0:
        directionX, directionY, directionZ = random_rotate_direction(-directionX, directionY, directionZ)
        birdX += directionX * speedX  # Move away from boundary
    if birdY >= 100 or birdY <= 0:
        directionX, directionY, directionZ = random_rotate_direction(directionX, -directionY, directionZ)
        birdY += directionY * speedY  # Move away from boundary
    if birdZ >= 100 or birdZ <= 0:
        directionX, directionY, directionZ = random_rotate_direction(directionX, directionY, -directionZ)
        birdZ += directionZ * speedZ  # Move away from boundary

    # Ensure the bird doesn't exceed boundaries
    birdX = np.clip(birdX, 0, 100)
    birdY = np.clip(birdY, 0, 100)
    birdZ = np.clip(birdZ, 0, 100)

    # Update the bird's position in the plot
    bird.set_data([birdX], [birdY])
    bird.set_3d_properties([birdZ])

    return bird,

# Create the animation
ani = FuncAnimation(fig, update, frames=200, interval=20, blit=False)

# Show the plot
plt.show()
