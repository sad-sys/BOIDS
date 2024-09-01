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

# Behaviour setting
avoidDistance = 10.0
neighbor_radius = 20
alignment_strength = 0.4
separation_strength = 0.6
cohesion_strength = 0.9

max_speed = 2.0  # Similar to the first implementation

# Limit speed function
def limit_speed(velocity, max_speed):
    speed = np.linalg.norm(velocity)
    if speed > max_speed:
        velocity = (velocity / speed) * max_speed
    return velocity

# Initialize the flock
for i in range(0, 50):
    flockX = np.append(flockX, random.randint(0, 100))
    flockY = np.append(flockY, random.randint(0, 100))
    flockZ = np.append(flockZ, random.randint(0, 100))

    flockDirectionX = np.append(flockDirectionX, random.choice([-1, 1]))
    flockDirectionY = np.append(flockDirectionY, random.choice([-1, 1]))
    flockDirectionZ = np.append(flockDirectionZ, random.choice([-1, 1]))

    bird, = ax.plot([flockX[i]], [flockY[i]], [flockZ[i]], marker=">", color='orange', markersize=10)
    birds.append(bird)

# Calculate speed based on distance to boundaries
def calculate_speed(position, boundary_min, boundary_max):
    distance_to_min = position - boundary_min
    distance_to_max = boundary_max - position
    distance = min(distance_to_min, distance_to_max)
    speed = max(0.1, (distance / 40))
    return speed

# Update function for animation
def update(frame):
    global flockX, flockY, flockZ, flockDirectionX, flockDirectionY, flockDirectionZ

    for i in range(50):
        # Calculate speeds based on distance to boundaries
        speedX = calculate_speed(flockX[i], 0, 100)
        speedY = calculate_speed(flockY[i], 0, 100)
        speedZ = calculate_speed(flockZ[i], 0, 100)

        velocity = np.array([flockDirectionX[i] * speedX, flockDirectionY[i] * speedY, flockDirectionZ[i] * speedZ])
        velocity = limit_speed(velocity, max_speed)

        # Update the bird's position
        flockX[i] += velocity[0]
        flockY[i] += velocity[1]
        flockZ[i] += velocity[2]

        # Check for boundary conditions and reverse direction if needed
        if flockX[i] >= 100 or flockX[i] <= 0:
            flockDirectionX[i] *= -1  # Reverse direction in X
            flockX[i] = np.clip(flockX[i], 0, 100)  # Ensure it stays within boundaries
            flockZ[i] += random.uniform(-1, 1)  # Small Z-axis nudge to avoid sticking

        if flockY[i] >= 100 or flockY[i] <= 0:
            flockDirectionY[i] *= -1  # Reverse direction in Y
            flockY[i] = np.clip(flockY[i], 0, 100)  # Ensure it stays within boundaries
            flockZ[i] += random.uniform(-1, 1)  # Small Z-axis nudge to avoid sticking

        if flockZ[i] >= 100 or flockZ[i] <= 0:
            flockDirectionZ[i] *= -1  # Reverse direction in Z
            flockZ[i] = np.clip(flockZ[i], 0, 100)  # Ensure it stays within boundaries

        ### Separation, Alignment, and Cohesion Behaviors
        steerVector = np.array([0.0, 0.0, 0.0])
        countSeparation = 0
        avgVelocity = np.array([0.0, 0.0, 0.0])
        countAlignment = 0
        centerOfMass = np.array([0.0, 0.0, 0.0])
        countCohesion = 0

        for j in range(50):
            if j == i:
                continue  # Skip itself

            birdPosElse = np.array([flockX[j], flockY[j], flockZ[j]])
            distance = np.linalg.norm(np.array([flockX[i], flockY[i], flockZ[i]]) - birdPosElse)

            # Separation behavior
            if distance < avoidDistance:
                force = (np.array([flockX[i], flockY[i], flockZ[i]]) - birdPosElse) / distance
                steerVector += force
                countSeparation += 1

            # Alignment behavior
            if distance < neighbor_radius:
                otherSpeedX = calculate_speed(flockX[j], 0, 100)
                otherSpeedY = calculate_speed(flockY[j], 0, 100)
                otherSpeedZ = calculate_speed(flockZ[j], 0, 100)
                otherVelocity = np.array([flockDirectionX[j] * otherSpeedX, flockDirectionY[j] * otherSpeedY, flockDirectionZ[j] * otherSpeedZ])

                avgVelocity += otherVelocity
                countAlignment += 1

                # Cohesion behavior: accumulate the positions of neighbors
                centerOfMass += birdPosElse
                countCohesion += 1

        # Apply separation steering
        if countSeparation > 0:
            steerVector /= countSeparation
            steerVector = steerVector / np.linalg.norm(steerVector) * 1.2  # Normalize

        # Apply alignment steering
        if countAlignment > 0:
            avgVelocity /= countAlignment
            avgVelocity = avgVelocity / np.linalg.norm(avgVelocity)  # Normalize

            steerVectorAlignment = avgVelocity - velocity / np.linalg.norm(velocity)
            steerVectorAlignment = steerVectorAlignment * alignment_strength

        # Apply cohesion steering
        if countCohesion > 0:
            centerOfMass /= countCohesion
            cohesionVector = centerOfMass - np.array([flockX[i], flockY[i], flockZ[i]])
            cohesionVector = cohesionVector / np.linalg.norm(cohesionVector) * 0.2

            velocity += (steerVector * separation_strength +
                         steerVectorAlignment * alignment_strength +
                         cohesionVector * cohesion_strength)

            velocity = velocity / np.linalg.norm(velocity)  # Normalize after adjustment

            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = velocity / np.linalg.norm(velocity)

        # Update the bird's position with adjusted direction
        flockX[i] += speedX * flockDirectionX[i]
        flockY[i] += speedY * flockDirectionY[i]
        flockZ[i] += speedZ * flockDirectionZ[i]

        # Update the bird's position in the plot
        birds[i].set_data([flockX[i]], [flockY[i]])
        birds[i].set_3d_properties([flockZ[i]])

    return ax.lines

# Create the animation
ani = FuncAnimation(fig, update, frames=200, interval=20, blit=False)

# Show the plot
plt.show()
