# AERO60492 – Autonomous Mobile Robots
## Coursework 2: Path Planning
**Author:** Euan Baldwin (10818421)

---

## Overview
This repository contains an implementation of the A* path planning algorithm for a two-dimensional grid environment, along with an interactive graphical user interface (GUI). Users can configure grid parameters, place obstacles, set start and end positions, and visualise the calculated optimal path in real-time.

---

## Files Included
- **`pathPlanner.py`**: Implements the core A* search algorithm.
- **`gui.py`**: Provides a PyQt5-based interactive GUI for visualisation and user interaction.

---

## Installation
### Requirements
- Python 3.x
- PyQt5 library

Install dependencies using pip:

```bash
pip install PyQt5
```

### Running the Application
Launch the GUI by running:

```bash
python gui.py
```

---

## Usage Instructions
Follow these steps to operate the application:

1. **Set Grid Dimensions**
   - Enter desired grid width and height.
   - Click `Reset` to apply dimensions.

2. **Add Obstacles**
   - Activate obstacle placement mode by clicking `Add Obstacles`.
   - Click or drag on grid cells to add obstacles.

3. **Set Start and End Points**
   - Click `Add Start` and select a cell as the starting position.
   - Click `Add End` and select a cell as the goal position.

4. **Run Path Planning**
   - Click `Run` to execute the A* algorithm.
   - The optimal path will be displayed on the grid.

Additional tools available:
- `Undo Obstacle`: Removes the most recent obstacle.
- `Clear`: Clears messages from the console area.

---

## Algorithm Details
- **Algorithm Type:** A* (informed search)
- **Grid Cell Values:** Open cell = `1`, Obstacle = `0`
- **Movement Cost:** Uniform (cost = `1` per move)
- **Allowed Movement Directions:** Up, Down, Left, Right
- **Heuristic Used:** Euclidean distance to goal
- **Algorithm Output:** List of coordinates `(col, row)` from start to end point.

---

## Error Handling and Validation
The application provides robust input validation and user feedback:

- Start/end positions must be within grid boundaries and unobstructed.
- If a valid path cannot be found, a clear error message is displayed in the GUI console.

---

## Acknowledgements
This project was developed for the MSc Robotics module AERO60492 – Autonomous Mobile Robots at the University of Manchester.
