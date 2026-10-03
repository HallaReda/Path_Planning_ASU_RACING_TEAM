## Path Planning Assignment


### Task Description

#### Part 1

You are given up to two cones (sometimes only one, or none), each with coordinates `(x, y)` and a `color` flag:
- `color == 0`: yellow cone (right side of the track)
- `color == 1`: blue cone (left side of the track)

You also receive the car pose `(x, y, yaw)`. Your goal is to return a sequence of path points `(x, y)` in world coordinates, representing a drivable route that stays between the left (blue) and right (yellow) boundaries. The path will be visualized by the tester.

Implement your algorithm in:

```
src/path_planning.py  ->  class PathPlanning.generatePath(self) -> Path2D
```

The environment already runs with a basic straight-line placeholder. Replace it with your solution.

#### Part 2

What if you were given three cones on one side of the track?
- Implement your approach to use the new givens.
- Add new test cases to test your approach in `src/scenarios.py`.
- Explain why you chose this solution and what could be its limitations.

### Quick Start

1. Install Python 3.9+.
2. Create a virtual environment (recommended) and install dependencies:

```
python -m venv .venv
source .venv/bin/activate  # macOS/Linux
# .venv/Scripts/activate   # Windows
pip install -r requirements.txt
```

3. Run a scenario:

```
python -m src.run --scenario 1
```

Available scenarios: numeric names `1`..`20` (each with up to 2 blue and 2 yellow cones placed on a 5x5 grid; car at (0,0) with varying yaw). Use `--scenario N` to select.

### Files Overview

- `src/models.py`: data classes for `Cone`, `CarPose`, and `Path2D` alias.
- `src/path_planning.py`: contains `PathPlanning` where you implement `generatePath`.
- `src/tester.py`: simple Matplotlib visualizer that plots cones, the car, heading, and the path.
- `src/scenarios.py`: prebuilt scenarios for testing.
- `src/run.py`: CLI to run a scenario and visualize the result.

### What to Submit

- Upload your solution to github and submit your repository link.
- Don't forget to add documentation, explaining your solution.

### Author's Note
- Keep it simple.
- This task was chosen to give you a sense of how things work, you don't have to implement complex algorithms, read research papers, or spend days thinking about it, just implement the simplest approach you can think of that can return a correct path.
- You are free to assume any missing information, but mention it in your submission.
- Try to solve as many cases as you can, but you don't have to solve all the problems that can ever exist, just try your best.
- Reminder: This problem is based on FSAI competition (a simplified version), it does not represent how path planning works everywhere it just provides an example to give you a better understanding of what path planning can look like.
- Enjoy :)


## Solution Approach

The solution uses a simple geometric approach rather than a complex path-planning algorithm.

The path is generated in world coordinates, starts at the car, and is made of points spaced at most 0.5 m apart. Its length is between 5 m and 10 m. It is built in three steps:
1. Pick a few key points (waypoints) depending on which cones are visible.
2. Fill in points between them every 0.5 m or less.
3. If the path is shorter than 5 m, extend it straight along its last direction (never beyond 10 m).

### Both sides detected

The closest blue and yellow cones are paired, and the midpoint of each pair is treated as a point on the centre of the track. The path goes from the car through these midpoints, nearest first.

### Only one side detected

**One cone:** the car's yaw is used as the travel direction. Blue is the left boundary, so the path is shifted to the right of it. Yellow is the right boundary, so the path is shifted to the left of it. The shift is half the assumed track width (1 m), and the path then continues forward.

**Two or more cones:** a straight line is fitted through the cones. Its direction is chosen so that the car is on the inside of the track (a blue line must have the car on its right, a yellow line must have the car on its left). The line is then shifted 1 m toward the middle of the track and followed.

### No cones detected

The car continues straight ahead along its current heading.

### Part 2: three cones on one side

With three or more cones on one side, I fit one straight line through all of them and shift it 1 m inward. I chose this because it is simple, and using all the cones means one badly placed cone matters less than when only two are available.

I added three scenarios to `src/scenarios.py`:
- **21:** three blue cones in a diagonal line
- **22:** three yellow cones in a horizontal line
- **23:** three blue cones and one yellow cone

Scenario 23 shows a limitation: when both sides are visible, only one blue and one yellow are paired, and the extra cones are ignored.

### Assumptions

- The track width is about 2 m.
- Over the short planning distance, a track boundary can be approximated by a straight line.
- The car is inside the track when only one boundary is visible.
- With a single cone, the car's yaw gives the direction of travel.
- The cone detections are correct and represent the nearby boundaries.

### Limitations

- A straight-line fit cannot follow sharp curves or irregular boundaries.
- When both sides are visible, unpaired cones are ignored (scenarios 7 and 23).
- The path goes in a straight line from the car to the first midpoint, so it can pass within 0.2-0.5 m of a cone (scenarios 2, 3, 6, 7, 10, 11, 19).
- A wrong track-width assumption shifts the path off the centre of the track.
- A real racing system would use more information and a dynamically feasible trajectory.

