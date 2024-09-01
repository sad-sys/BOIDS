# BOIDS
# 3D Boid Simulation

## Overview

This project simulates the flocking behavior of birds using a 3D space. The simulation is built in Python using `matplotlib` and implements key aspects of flocking behavior such as separation, alignment, and cohesion. The birds move within a bounded 3D space, adjusting their velocity and direction based on the positions and movements of their neighbors, while also avoiding the boundaries of the space.

## Features

- **3D Visualization:** The simulation is visualized in a 3D space using `matplotlib`'s `Axes3D`.
- **Flocking Behavior:** Implements separation, alignment, and cohesion behaviors, typical of flocking animals.
- **Boundary Handling:** Birds will turn away when they reach the boundaries of the space to remain within bounds.
- **Speed Limiting:** Ensures that birds do not exceed a maximum speed.
- **Random Directional Changes:** Birds make slight random adjustments to their direction to simulate natural movement.
