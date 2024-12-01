# Incremental RRT with Dubins

A motion planning algorithm for non-holonomic autonomous agents, such as robots or drones, that require turning constraints. This repository provides implementations for calculating **Dubins paths**, **RRT with Dubins**, **RRT\* with Dubins**, and **Incremental RRT with Dubins** to help agents navigate efficiently and optimally.

### What’s inside:
- **Dubins Path Calculation**: Efficient calculation of the shortest path between two points for non-holonomic agents.
- **RRT with Dubins**: Rapidly exploring random tree (RRT) for motion planning using Dubins paths.
- **RRT\* with Dubins**: Optimal version of RRT that refines the path for better efficiency.
- **Incremental RRT with Dubins**: A faster variant of RRT\* that refines the path over time.

While calculating Dubins paths manually may seem useful, there's already a well-established **Dubins library** for shortest path calculation, making this implementation more for educational purposes.
## Overview

A motion planning algorithm specifically designed for non-holonomic autonomous agents, where turning constraints must be respected. This repository includes multiple variations of the **RRT (Rapidly-exploring Random Tree)** algorithm, each tailored to account for Dubins paths, which are ideal for vehicles with turning limitations.

### Why Dubins Path?
- Dubins paths offer the shortest possible route between two points for vehicles with a turning radius constraint.
- Common in autonomous vehicle motion planning (e.g., drones, self-driving cars).

## How the Algorithm Works

The **Incremental RRT with Dubins** builds upon traditional RRT by progressively improving the path with each iteration. By using the Dubins path calculation, it ensures that turning constraints are respected throughout the process.

![Algorithm Workflow](https://github.com/user-attachments/assets/c1e9f0d2-0c18-47d5-8e85-0b58488ca697)

## Final Trajectory

Here’s an example of the final trajectory generated using Incremental RRT with Dubins:

![Final Trajectory](https://github.com/user-attachments/assets/0245f9f5-0786-480f-9c7c-b28cd91a250e)
