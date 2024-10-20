# ROS2 Turtlesim: The Turtle Terminator

![image](https://github.com/user-attachments/assets/0b1368a2-2074-4fb1-a694-66b68de0117e)

This repository contains two ROS2 nodes that interact with the `turtlesim` simulator. One node is responsible for spawning turtles at random positions, and the other controls a turtle to chase and "capture" the spawned turtles.

## Description

- **Spawner Node**: Generates turtles at random positions in the `turtlesim` simulator and keeps track of the alive turtles. It publishes a list of alive turtles on the `alive_turtles` topic and provides a `turtle_taken` service to remove turtles from the simulator.

- **Controller Node**: Controls the main turtle (`turtle1`) to chase the turtles generated by the `SpawnerNode`. When the main turtle gets close enough to a spawned turtle, it requests the `turtle_taken` service to remove the captured turtle.

![Demo](https://github.com/user-attachments/assets/cc9e82dd-69d4-4053-91a1-4166b7e1a548)

## Requirements

- ROS2 Humble (or another compatible distribution)
- Python 3
- ROS2 `turtlesim` package
- Create a custom interface package including custom messages and services that are loaded into the repository.

## Objectives
The objective of this project is the implementation of ROS2 concepts such as:
- Creation of nodes.
- Creation of custom interfaces, including messages and services.
- Use of already created interfaces such as those of ‘turtlesim’.

And other tools not included in this repository such as:
- Configuration of parameters in the nodes.
- Creation of a launch file to run all nodes.

Parameters allow customisation of node-internal variables during execution, including in the launch file, so as research from this repository, it is recommended to investigate these definitions.

## Nodes

### SpawnerNode

#### Purpose: The SpawnerNode is responsible for spawning new turtles in the turtlesim simulator at random positions and orientations. It keeps track of all the alive turtles and publishes this information so other nodes can access it. Additionally, it provides a service to remove (kill) turtles from the simulator when they have been "captured" by the controller node.

#### Key Functionalities:

- Spawning Turtles: Periodically spawns new turtles using the /spawn service provided by turtlesim. Each turtle is assigned a unique name (e.g., turtle_2, turtle_3, etc.) and random x, y, and theta values within the simulator's boundaries.

- Tracking Alive Turtles: Maintains a list (alive_turtles_) of all currently alive turtles. This list is updated whenever a new turtle is spawned or when a turtle is removed from the simulator.

- Publishing Alive Turtles: Publishes the list of alive turtles on the alive_turtles topic using a custom message type TurtleArray. This allows other nodes, like the ControllerNode, to know which turtles are available to chase.

- Service to Remove Turtles: Offers a service named turtle_taken (using a custom service type TurtleTaken) that allows other nodes to request the removal of a specific turtle by name. When this service is called, the node uses the /kill service to remove the turtle from the simulator and updates the list of alive turtles accordingly.

#### Implementation Details:

- Timers and Callbacks:

  - Spawner Timer: Uses a timer (spawner_timer) to call the spawn_function every second, which initiates the spawning of a new turtle.
  - Spawn Function: Generates random positions and orientations for the new turtle and calls the call_spawn_server method to spawn it.
  - Spawn Callback: After the turtle is spawned, the callback_call_spawn method adds the new turtle to the list of alive turtles and publishes the updated list.

- Service Clients:

  - Spawn Service: Implements a client to call the /spawn service asynchronously.
  - Kill Service: Implements a client to call the /kill service asynchronously when a turtle needs to be removed.
    
- Service Servers:

  - Turtle Taken Service: Provides the turtle_taken service to handle requests to remove turtles.

- Custom Messages and Services:

  - Turtle.msg: Represents individual turtle information (name, x, y, theta).
  - TurtleArray.msg: An array of Turtle messages representing all alive turtles.
  - TurtleTaken.srv: Service definition for removing a turtle by name.

### ControllerNode
#### Purpose: The ControllerNode controls the main turtle (turtle1) in the turtlesim simulator to chase and capture the turtles spawned by the SpawnerNode. It calculates the necessary movement commands to navigate towards the target turtle and requests the removal of the turtle once it is "captured".

#### Key Functionalities:

- Subscribing to Alive Turtles: Subscribes to the alive_turtles topic to receive updates on all alive turtles' positions and names. This information is used to determine which turtle to chase.

- Selecting a Target Turtle:

  - Hunt Closest Turtle: By default, the node selects the closest turtle to turtle1 to chase.
  - Target Selection Logic: Calculates the distance to each alive turtle and updates the target (turtle_hunt_) when a closer turtle is found.

- Controlling Turtle Movement:

  - Pose Subscription: Subscribes to turtle1/pose to get real-time updates on turtle1's position and orientation.
  - Control Loop: Uses a high-frequency timer (control_loop_timer_) to execute the control_loop method, which calculates and publishes velocity commands.
  - Velocity Calculation: Computes linear and angular velocities based on the distance and angle to the target turtle, ensuring smooth and efficient movement towards it.

- Capturing Turtles:

  - Proximity Check: When turtle1 is within a certain threshold distance (e.g., 0.4 units) of the target turtle, it considers the turtle captured.
  - Service Call: Calls the turtle_taken service to request the removal of the captured turtle.
  - Resetting Target: After a turtle is captured, resets the target (turtle_hunt_) to None until a new target is selected.

#### Implementation Details:

- Subscribers:

  - Pose Subscriber: Receives updates on turtle1's current position and orientation.
  - Alive Turtles Subscriber: Receives updates on the list of alive turtles to determine which turtle to chase.

- Publishers:

  - Command Velocity Publisher: Publishes Twist messages to turtle1/cmd_vel to control turtle1's movement.

- Control Logic:

  - Distance Calculation: Calculates the Euclidean distance between turtle1 and the target turtle.
  - Orientation Adjustment: Computes the required angular velocity to align turtle1 towards the target turtle.
  - Speed Adjustment: Adjusts the linear velocity proportionally to the distance, allowing for acceleration and deceleration as needed.

- Service Clients:

  - Turtle Taken Service: Implements a client to call the turtle_taken service asynchronously when a turtle is captured.

- Custom Messages and Services:

  - Utilizes the same Turtle, TurtleArray, and TurtleTaken custom interfaces defined in the my_interfaces package.

- Behavior Summary:

The ControllerNode continuously monitors the positions of alive turtles and navigates turtle1 towards the closest one.
When turtle1 captures a turtle, it communicates with the SpawnerNode to remove that turtle from the simulator.
This process repeats as new turtles are spawned by the SpawnerNode, creating an ongoing game of chase within the turtlesim simulator.
