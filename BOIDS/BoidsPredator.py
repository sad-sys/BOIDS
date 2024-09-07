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

predatorX = 0
predatorY = 0
predatorZ = 0

flockDirectionX = np.array([])  # 1 for increasing X, -1 for decreasing X
flockDirectionY = np.array([])  # 1 for increasing Y, -1 for decreasing Y
flockDirectionZ = np.array([])  # 1 for increasing Z, -1 for decreasing Z

predatorDirectionX = np.array([])
predatorDirectionY = np.array([])
predatorDirectionZ = np.array([])


birds = []

#Behaviour setting
avoidDistance = 5.0
neighbor_radius = 20
alignment_strength = 0.1

alignment_strength = 0.3
separation_strength = 0.6
cohesion_strength = 0.6

max_speed = 3.0  # Similar to the first implementation

predatorRepulson = 1.0

# Set a smoothing factor for the predator's movement
predator_smoothing_factor = 0.05  # Lower value means smoother (slower) transitions

# Initialize predator velocity as a separate variable to maintain inertia
predator_velocity = np.array([0.0, 0.0, 0.0])

# Replace the calculate_speed function with a fixed speed control:
def limit_speed(velocity, max_speed):
    speed = np.linalg.norm(velocity)
    if speed > max_speed:
        velocity = (velocity / speed) * max_speed
    return velocity

for i in range(0,50):
    flockX = np.append(flockX, random.randint(0,100))
    flockY = np.append(flockY, random.randint(0,100))
    flockZ = np.append(flockZ, random.randint(0,100))

    flockDirectionX = np.append(flockDirectionX, random.choice([-1, 1]))
    flockDirectionY = np.append(flockDirectionY, random.choice([-1, 1]))
    flockDirectionZ = np.append(flockDirectionZ, random.choice([-1, 1]))

    bird, = ax.plot([flockX[i]], [flockY[i]], [flockZ[i]], marker=">", color='orange', markersize=10)
    birds.append(bird)

predator, = ax.plot([predatorX], [predatorY], [predatorZ], marker=">", color='red', markersize=10)

def calculate_speed(position, boundary_min, boundary_max):
    distance_to_min = position - boundary_min
    distance_to_max = boundary_max - position
    distance = min(distance_to_min, distance_to_max)
    
    # Adjust the divisor to slow down the bird's speed
    speed = max(0.1, (distance / 40))
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

def update(frame):
    global flockX, flockY, flockZ, flockDirectionX, flockDirectionY, flockDirectionZ, predatorX, predatorY, predatorZ, predatorDirectionX, predatorDirectionY, predatorDirectionZ, predator_velocity

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
            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = random_rotate_direction(-flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i])
            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = random_rotate_direction(-flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i])
        if flockY[i] >= 100 or flockY[i] <= 0:
            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = random_rotate_direction(flockDirectionX[i], -flockDirectionY[i], flockDirectionZ[i])
            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = random_rotate_direction(-flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i])
        if flockZ[i] >= 100 or flockZ[i] <= 0:
            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = random_rotate_direction(flockDirectionX[i], flockDirectionY[i], -flockDirectionZ[i])
            flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i] = random_rotate_direction(-flockDirectionX[i], flockDirectionY[i], flockDirectionZ[i])

        # Ensure the bird doesn't exceed boundaries
        flockX[i] = np.clip(flockX[i], 0, 100)
        flockY[i] = np.clip(flockY[i], 0, 100)
        flockZ[i] = np.clip(flockZ[i], 0, 100)

        ### Separation Behavior
        steerVector = np.array([0.0, 0.0, 0.0])
        countSeparation = 0

        ### Alignment Behavior
        avgVelocity = np.array([0.0, 0.0, 0.0])
        countAlignment = 0

        ### Cohesion Behavior
        centerOfMass = np.array([0.0, 0.0, 0.0])
        countCohesion = 0

        ### Repulsion Behaviour
        repulseVector = np.array([0.0, 0.0, 0.0])
        countRepulse = 0

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

            # Repulsion Behaviour
            predatorPos = np.array([predatorX, predatorY, predatorZ])
            predatorDistance = np.linalg.norm(np.array([flockX[i], flockY[i], flockZ[i]]) - predatorPos)

            if predatorDistance < 10:
                force = (np.array([flockX[i], flockY[i], flockZ[i]]) - predatorPos) / predatorDistance
                repulseVector += force

        # Apply separation steering
        if countSeparation > 0:
            steerVector /= countSeparation
            steerVector = steerVector / np.linalg.norm(steerVector) * 1.2  # Normalize

        #Apply predator Repulson steering
        repulseVector = repulseVector / np.linalg.norm(repulseVector) * 1.2

        # Apply alignment steering
        if countAlignment > 0:
            avgVelocity /= countAlignment
            avgVelocity = avgVelocity / np.linalg.norm(avgVelocity)  # Normalize

            # Calculate the alignment steering vector
            steerVectorAlignment = avgVelocity - velocity / np.linalg.norm(velocity)
            steerVectorAlignment = steerVectorAlignment * alignment_strength  # Apply alignment strength

        # Apply cohesion steering
        if countCohesion > 0:
            centerOfMass /= countCohesion  # Calculate center of mass
            cohesionVector = centerOfMass - np.array([flockX[i], flockY[i], flockZ[i]])  # Direction towards center of mass
            cohesionVector = cohesionVector / np.linalg.norm(cohesionVector) * 0.2 # Normalize the cohesion vector

            # Combine separation, alignment, and cohesion
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

     # Calculate the center of mass for the birds
    centerOfMass = np.array([np.mean(flockX), np.mean(flockY), np.mean(flockZ)])

    # Calculate the direction towards the center of mass from the predator's current position
    direction_to_com = centerOfMass - np.array([predatorX, predatorY, predatorZ])

    # Normalize the direction vector
    direction_to_com /= np.linalg.norm(direction_to_com)

    # Calculate predator's target velocity based on direction to the center of mass
    predatorSpeedX = min(1.5, calculate_speed(predatorX, 0, 100))
    predatorSpeedY = min(1.5, calculate_speed(predatorY, 0, 100))
    predatorSpeedZ = min(1.5, calculate_speed(predatorZ, 0, 100))

    target_velocity = np.array([direction_to_com[0] * predatorSpeedX, direction_to_com[1] * predatorSpeedY, direction_to_com[2] * predatorSpeedZ])

    # Smoothly transition to the new velocity using the smoothing factor
    predator_velocity = predator_velocity * (1 - predator_smoothing_factor) + target_velocity * predator_smoothing_factor

    # Limit the predator's speed
    predator_velocity = limit_speed(predator_velocity, max_speed)

    # Update the predator's position based on the new smoothed velocity
    predatorX += predator_velocity[0]
    predatorY += predator_velocity[1]
    predatorZ += predator_velocity[2]

    # Ensure predator doesn't exceed boundaries
    predatorX = np.clip(predatorX, 0, 100)
    predatorY = np.clip(predatorY, 0, 100)
    predatorZ = np.clip(predatorZ, 0, 100)

    # Update predator's position in the plot
    predator.set_data([predatorX], [predatorY])
    predator.set_3d_properties([predatorZ])

    return ax.lines

# Create the animation
ani = FuncAnimation(fig, update, frames=200, interval=20, blit=False)

# Show the plot
plt.show()