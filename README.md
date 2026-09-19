*This project has been created as part of the 42 curriculum by \<mecharra>.*

# Fly-in

## Description

Fly-in is a Python simulation that models the movement of multiple drones through a network of interconnected hubs. Each drone starts from the same starting hub and must reach the destination hub while respecting the constraints defined by the map.

The simulator supports different types of hubs and connections, including:

* Normal hubs
* Blocked hubs
* Restricted hubs
* Priority hubs
* Hub capacity limits
* Connection capacity limits

The objective is to deliver every drone to the destination while avoiding collisions, respecting capacities, and choosing efficient routes.

---

## Features

* Map parser with syntax validation
* Graph representation of hubs and connections
* Dijkstra-based pathfinding
* Multi-drone scheduler
* Support for blocked, restricted and priority hubs
* Zone occupancy management
* Connection capacity management
* ANSI color visualization
* Error handling for invalid maps and unreachable destinations

---

## Algorithm and Implementation Strategy

The project is divided into several independent components.

### Parser

The parser reads the map file, validates its syntax and creates the graph used by the simulator.

It verifies:

* valid map format
* duplicate hubs
* duplicate connections
* undefined hubs
* invalid metadata
* missing start or end hub

Once validated, it creates all drone objects at the starting hub.

### Graph

The graph stores every hub and connection.

Each hub contains:

* coordinates
* type
* maximum drone capacity
* optional display color

Each connection stores the two connected hubs and its maximum link capacity.

### Pathfinding

Routing is performed using Dijkstra's algorithm.

Each hub has a movement cost:

* Normal hub: cost 1
* Priority hub: cost 1
* Restricted hub: cost 2
* Blocked hub: not traversable

The pathfinder also considers:

* occupied hubs
* future congestion using planned path penalties
* priority hubs during tie-breaking

If no valid path exists, a `PathNotFoundError` is raised.

### Scheduling

The scheduler resolves drone movements turn by turn.

For every simulation step it:

1. Computes each drone's intended move.
2. Groups drones requesting the same destination.
3. Checks destination hub capacities.
4. Checks connection capacities.
5. Approves valid moves.
6. Recomputes alternate paths for rejected drones whenever possible.
7. Generates the final list of moves for the current turn.

This approach prevents collisions while maximizing the number of drones that can move simultaneously.

### Simulation

The simulator executes the approved moves until every drone reaches the destination.

It updates:

* drone positions
* hub occupancy
* restricted-zone transit
* drone status
* visualization output

The simulation finishes when all drones are successfully delivered.

## Visual Representation

The simulator prints the result of every simulation turn.

Each output line contains the drones that successfully moved during that turn.

Example:

```text
TURN1: D1-junction D2-junction
TURN2: D1-path_a D2-path_b
TURN3: D1-goal D2-goal
```

The visualizer supports ANSI terminal colors.

Zones may define a color using metadata:

```text
hub: checkpoint 4 2 [color=blue]
```

When supported by the terminal, the hub name is displayed using its assigned color, making it easier to distinguish important areas such as the start hub, destination hub, or special zones.

When a drone is travelling through a restricted hub, the visualizer displays the connection name instead of the hub name, allowing the user to identify the occupied connection.

The visualization is entirely terminal-based and requires no external graphical libraries.

## Example

### Input

```text
nb_drones: 2

start_hub: start 0 0 [color=green]
hub: waypoint1 1 0
hub: waypoint2 2 0
end_hub: goal 3 0 [color=red]

connection: start-waypoint1
connection: waypoint1-waypoint2
connection: waypoint2-goal
```

### Expected Output

```text
TURN1: D1-waypoint1
TURN2: D1-waypoint2 D2-waypoint1
TURN3: D1-goal D2-waypoint2
TURN4: D2-goal

Simulation complete in 4 turns.
2 drone(s) delivered to 'goal'.
```

The exact output may differ depending on the chosen scheduling policy when multiple drones compete for the same hub or connection, but every simulation guarantees that capacity constraints are respected and all reachable drones eventually arrive at the destination.

## Instructions

### Requirements

* Python 3.11 or newer

### Install dependencies

```bash
make install
```

or manually

```bash
python3 -m pip install mypy flake8
```

### Run

```bash
make run
```

To run another map:

```bash
make run MAP=maps/easy/02_simple_fork.txt
```

### Debug

```bash
make debug
```

### Static analysis

```bash
make lint
```

### Clean generated files

```bash
make clean
```

---

## Project Structure

```
.
├── main.py
├── parser.py
├── simulator.py
├── models.py
├── visualisor.py
├── maps/
├── Makefile
└── README.md
```

---

## Resources

### Documentation

* Python Documentation — https://docs.python.org/3/
* heapq module — https://docs.python.org/3/library/heapq.html
* Enum module — https://docs.python.org/3/library/enum.html
* Typing module — https://docs.python.org/3/library/typing.html

### Algorithms

* Dijkstra's Shortest Path Algorithm
* Graph Theory
* Priority Queues (Binary Heap)

### AI Usage

ChatGPT was used during the development of this project for:

* explaining graph theory concepts
* understanding Dijkstra's algorithm
* discussing scheduling strategies
* reviewing code for potential bugs
* improving code organization
* generating and refining the project documentation

All design decisions, implementation, testing and final validation were performed by the project author.
