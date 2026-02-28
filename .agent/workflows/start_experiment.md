---
description: Start a new experiment for an algorithm
---

# Workflow: Start New Experiment

1. **Identify Source**: Locate the desired algorithm in `PythonRobotics/` (e.g., `PythonRobotics/PathPlanning/AStar/a_star.py`).
2. **Create Experiment File**: 
   - Create a new file in `PythonRoboticsLearning/experiments/` (e.g., `astar_variant_01.py`).
   - Add standard boilerplate to setup path:
     ```python
     import sys
     import os
     # Add PythonRobotics root to path
     sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../PythonRobotics")))
     ```
3. **Link Note**: Check if a note exists in `PythonRoboticsLearning/notes/`. If not, create one.
4. **Implementation**:
   - Either import the original class/functions.
   - OR copy the specific code block needed for modification.
5. **Verify**: Run the new experiment file to ensure imports work.
