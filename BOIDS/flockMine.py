import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import random

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_zlim(0, 100)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

flockX = np.array([])
flockY = np.array([])
flockZ = np.array([])

flockDirectionX = np.array([])  # 1 for increasing X, -1 for decreasing X
flockDirectionY = np.array([])  # 1 for increasing Y, -1 for decreasing Y
flockDirectionZ = np.array([])  # 1 for increasing Z, -1 for decreasing Z

birds = []


for i in range(0,50):
    flockX = np.append(flockX, random.randint(0,25))
    flockY = np.append(flockX, random.randint(0,25))
    flockZ = np.append(flockX, random.randint(0,25))

    flockDirectionX = np.append(flockDirectionX, random.choice([-1, 1]))
    flockDirectionY = np.append(flockDirectionY, random.choice([-1, 1]))
    flockDirectionZ = np.append(flockDirectionZ, random.choice([-1, 1]))

    bird, = ax.plot([flockX[i]], [flockY[i]], [flockZ[i]], marker=">", color='orange', markersize=10)
    birds.append(bird)


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
    global flockX, flockY, flockZ, flockDirectionX, flockDirectionY, flockDirectionZ

    for i in range(0,50):
    # Calculate speeds based on distance to boundaries
        speedX = calculate_speed(flockX[i], 0, 100)
        speedY = calculate_speed(flockY[i], 0, 100)
        speedZ = calculate_speed(flockZ[i], 0, 100)

        # Update the bird's position
        flockX[i] += speedX * flockDirectionX[i]
        flockY[i] += speedY * flockDirectionY[i]
        flockZ[i] += speedZ * flockDirectionZ[i]

            # Check for boundary conditions and reverse direction if needed
        if flockX[i] >= 100 or flockX[i] <= 0:
            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = random_rotate_direction(-flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i])
        if flockY[i] >= 100 or flockY[i] <= 0:
            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = random_rotate_direction(flockDirectionX[i], -flockDirectionY[i], flockDirectionZ[i])
        if flockZ[i] >= 100 or flockZ[i] <= 0:
            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = random_rotate_direction(flockDirectionX[i], flockDirectionY[i], -flockDirectionZ[i])


        # Ensure the bird doesn't exceed boundaries
        flockX[i] = np.clip(flockX[i], 0, 100)
        flockY[i] = np.clip(flockY[i], 0, 100)
        flockZ[i] = np.clip(flockZ[i], 0, 100)

        # Update the bird's position in the plot
        # Update the bird's position in the plot
        birds[i].set_data([flockX[i]], [flockY[i]])
        birds[i].set_3d_properties([flockZ[i]])

    return ax.lines
# Create the animation
ani = FuncAnimation(fig, update, frames=200, interval=20, blit=False)

# Show the plot
plt.show()