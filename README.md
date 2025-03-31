# Research Track I Assignment_2_Part_1

This repository contains the solution to the "Research Track I" assignment, which focuses on implementing long-running tasks using an action server. The tasks are structured into three nodes as described below, along with a launch file to start the simulation.

## Assignment Description

The primary problem addressed is the blocking nature of the robot task while reaching a target. To overcome this, the implementation utilizes an action server provided in the `assignment_2_2024` package. The assignment involves:

- Creating a new package.
- Developing three nodes:
  - An action client node to interact with the action server.
  - A service node to return the last target's coordinates.
 
- Creating a launch file to start the simulation.

## Implementation

### Nodes

#### Action Client Node

- Allows the user to set a target (x, y) or cancel it.
- Utilizes feedback/status from the action server to determine when the target is reached.
- Publishes robot position and velocity as a custom message `(x, y, vel_x, vel_z)` using data from the `/odom` topic.

#### Service Node

- Provides a service to return the coordinates of the last target set by the user.


### Launch File

- Starts all nodes and the simulation environment.

## Steps to Run

1. **Clone this repository:**
   ```bash
   git clone https://github.com/MahmoudElderiny/assignment2_rt.git

  2. **Navigate to the package directory:**
   ```bash
   cd assignment2_rt
```
3. **Build the package:**
```bash
catkin_make
```
4. **Source the workspace:**
```bash
source devel/setup.bash
```
5.**Launch the simulation:**
```bash
roslaunch assignment2_rt simulation.launch

```
## Interact with the nodes:
 - Use the action client to set or cancel targets.
 - Call the service node to get the last target coordinates.
 - Monitor robot position and velocity updates.
   



 

