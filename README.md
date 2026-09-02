# Artificial Vision Maze Robot

A small, dependency-free C++20 robot simulation. It is deliberately terminal-based so the sensing, planning, and movement loop is easy to inspect before adding a graphics or computer-vision library.

## Current behaviour

- Generates the same 31 by 21 perfect maze on each run, making experiments reproducible.
- Uses A* to route the robot from the top-left start to the bottom-right goal.
- Simulates left, front, and right range readings with Gaussian measurement noise.
- Renders a small forward-facing symbolic camera image (`#` is a wall, `.` is floor, and `G` is the goal).
- Animates the route and robot heading in an ANSI-compatible terminal.

The first version deliberately gives the A* planner the complete maze so that motion, path planning, and sensors can be verified independently. The next milestone is to build an occupancy grid only from sensor and camera measurements, then re-plan from that discovered map.

## Build and run

Open a **Developer PowerShell for Visual Studio**, then run:

```powershell
cmake --preset x64-debug
cmake --build out/build/x64-debug --target ArtificialVison
.\out\build\x64-debug\ArtificialVison\ArtificialVison.exe
```

For a fast non-animated run:

```powershell
.\out\build\x64-debug\ArtificialVison\ArtificialVison.exe --delay 0
```

Useful options:

- `--delay <milliseconds>` controls time between motion frames (default: 110).
- `--steps <count>` stops after a number of frames, which is useful for inspecting the early sensor readings.

## Suggested next increments

1. Add an `OccupancyGrid` with unknown, free, and occupied cells.
2. Ray-cast each range measurement into that grid and update it with confidence values.
3. Plan only through known-free cells; use frontier exploration when the goal is unknown.
4. Replace the symbolic camera with a raylib scene and an OpenCV image-processing pipeline.
