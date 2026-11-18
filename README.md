# Autonomous Navigation Simulator

An interactive 2D mobile-robot planning demonstration with **A***, **Dijkstra**, a clickable obstacle editor, and dynamic replanning. Implemented using the Python standard library; no robotics hardware required.

## Features
- Four-connected grid planning with shortest paths and deterministic tie-breaking
- Graceful handling of blocked/unreachable goals
- Robot state and obstacles updated during simulation
- Tkinter map editor and animation (desktop Python with Tk support)
- Scriptable ASCII-map command-line interface; automated unit tests

## Run
```bash
python main.py --map maps/warehouse.txt
python main.py --algorithm dijkstra
python main.py --gui
python -m unittest discover -s tests -v
```
Requires Python 3.10+. `--gui` requires Tkinter and a display. All CLI features work headlessly.

## Map legend
`S` start, `G` goal, `#` obstacle, `.` free cell. Four-neighbor movements cost one unit.

## Architecture
`navigation.py` isolates the model, algorithms, and robot from `main.py` (CLI and visualization). This makes it possible to replace grid neighbors with motion primitives or plug in alternative search algorithms.

## Engineering notes
A* with Manhattan heuristic is admissible and consistent for this unit-cost four-neighbor graph; Dijkstra provides a baseline. Current dynamic replanning recomputes a complete path each step and is deliberately not D* Lite. This is a grid-level teaching simulator, not physical vehicle-control software.

## Ideas for extensions
Continuous dynamics, A* with turn penalties, mapping from simulated LiDAR, D* Lite and ROS 2 integration.

MIT licensed. See `LICENSE`.
