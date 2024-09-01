import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Set up the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set the initial position of the ball
ballX = 25
ballY = 25
ballZ = 25

# Parameters for the simulation
gravity = 10
ballSpeed = 0
delta_t = 0.1  # Time step
energy_loss_factor = 0.9  # Factor by which the speed is reduced after each bounce

# Set the limits of the plot
ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.set_zlim(0, 50)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Plot the initial position of the ball
ball, = ax.plot([ballX], [ballY], [ballZ], 'bo', markersize=10)

def update(frame):
    global ballZ, ballSpeed, gravity

    # Apply gravity to the ball's speed
    ballSpeed -= gravity * delta_t
    
    # Update the ball's position based on its speed
    ballZ += ballSpeed * delta_t

    # Check for bounce on the ground (z=0 plane)
    if ballZ <= 0:
        ballZ = 0  # Ensure the ball doesn't go below ground
        ballSpeed = -ballSpeed * energy_loss_factor  # Reverse and reduce the speed

    # Update the position of the ball
    ball.set_data([ballX], [ballY])
    ball.set_3d_properties([ballZ])

    print(ballX,ballY,ballZ)

    return ball,

# Create the animation
ani = FuncAnimation(fig, update, frames=200, interval=20, blit=True)

# Show the plot
plt.show()
