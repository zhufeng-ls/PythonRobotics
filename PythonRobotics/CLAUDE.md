# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Commands

### Environment Setup
- **Conda (recommended)**: `conda env create -f requirements/environment.yml`
- **Pip**: `pip install -r requirements/requirements.txt`
- **Python version**: Requires Python 3.13.x

### Testing
- **Run all tests**: `python3 -m pytest`
- **Run single test**: `python3 -m pytest tests/test_a_star.py::test_1 -v`
- **Run tests in parallel**: `python3 -m pytest -n auto`
- **Test with warnings as errors**: Tests run with `-W error` flag by default

### Code Quality
- **Type checking**: `mypy .`
- **Linting**: `ruff check .`
- **Format**: `ruff format .`
- **Run pytest directly**: Individual test files can be run as scripts (use `python tests/test_a_star.py`)

### Documentation
- **Build docs**: `cd docs && make html`
- **View local docs**: Open `docs/_build/html/index.html`

## Architecture Overview

### Project Structure
PythonRobotics is an **educational robotics algorithm library** with 100+ standalone algorithm implementations organized by robotics domain:

- **PathPlanning/**: 48 path planning algorithms (Grid search, Sampling-based, Curve generation, Roadmaps, Reactive, Coverage)
- **PathTracking/**: 8 control algorithms (LQR, MPC, Stanley, Pure Pursuit, etc.)
- **Localization/**: 6 filtering algorithms (EKF, UKF, Particle Filter, etc.)
- **SLAM/**: 5 SLAM implementations (EKF-SLAM, FastSLAM, Graph-SLAM, ICP)
- **Mapping/**: 13 mapping algorithms (Grid maps, Clustering, Fitting)
- **ArmNavigation/**: 5 robotic arm algorithms with obstacle avoidance
- **AerialNavigation/**: Drone trajectory and rocket landing
- **Other**: Bipedal, InvertedPendulum, MissionPlanning

### Algorithm Design Pattern
Each algorithm follows a **consistent educational structure**:

1. **Standalone executable file** (e.g., `a_star.py`, `rrt_star.py`)
2. **Class-based implementation** with clear initialization parameters
3. **`main()` function** for direct execution and visualization
4. **Animation flag** (`show_animation`) to toggle matplotlib visualization
5. **Minimal external dependencies** - only NumPy, SciPy, Matplotlib, CVXPY

Example structure:
```python
class AlgorithmPlanner:
    def __init__(self, params...):
        # Initialize algorithm-specific parameters

    def planning(self, start, goal):
        # Core planning algorithm
        return path

def main():
    # Demo with animation
    if show_animation:
        plt.plot(...)
```

### Code Organization Principles

#### Module Structure
- **No complex package hierarchy** - algorithms are directly importable via module paths
- **Shared utilities in `utils/`** for common mathematical functions
- **Each algorithm self-contained** with visualization integrated
- **Consistent variable naming** across modules (sx, sy = start, gx, gy = goal)

#### Testing Architecture
- **Dedicated test file per algorithm** in `tests/` directory
- **Simple test pattern**:
  ```python
  import algorithm_module as m
  def test_1():
      m.show_animation = False  # Disable animation for CI
      m.main()
  ```
- **`conftest.py`** sets up PYTHONPATH for importing modules
- **Tests import directly from source** (no test fixtures expected)

#### Documentation System
- **Sphinx-based textbook** at `docs/` with algorithm explanations
- **Online documentation**: https://atsushisakai.github.io/PythonRobotics/
- **Algorithm categories** have dedicated docs sections
- **Mathematical backgrounds** and references included

### Key Implementation Patterns

#### Animation Control
Most algorithms support animation toggling:
```python
show_animation = True/False  # Global flag
if show_animation:
    plt.plot(...)  # Visualization code
```

#### Coordinate Systems
- **2D grid-based** algorithms use (x, y) coordinates
- **Robot position**: (sx, sy) = start, (gx, gy) = goal
- **Obstacles**: ox[], oy[] arrays for obstacle positions
- **Grid resolution**: configurable for discretization

#### Path Representation
- **Paths returned as coordinate arrays**: [x_list], [y_list]
- **Some algorithms include orientation/velocity**: [x_list], [y_list], [yaw_list], [v_list]

### Development Culture
- **Educational focus** prioritizes understandability over performance
- **Minimal dependencies** ensure easy installation
- **Consistent visualization** using Matplotlib animations
- **Academic references** included with each algorithm
- **Active maintenance** with recent updates to dependencies

### Running Algorithms Individually
```python
# Run any algorithm directly
cd PathPlanning/AStar/
python3 a_star.py

# Or as module from project root
python3 -m PathPlanning.AStar.a_star
```

### Important Notes
- **Cursor rules (.cursorrules)** are for Chinese note-taking system - NOT relevant to PythonRobotics development
- **No build system required** - pure Python with pip/conda dependencies
- **No formal API** - each algorithm is designed as standalone example
- **Matplotlib animations** are integral to educational value - preserve when modifying algorithms

## Interactive Demo Development Guidelines

When creating interactive algorithm demonstrations with matplotlib widgets:

### Variable Scoping in f-strings
- **Always verify all variables in f-strings are defined** before the string is evaluated
- Common mistake: Using loop variables from different scopes (e.g., `{y}` when loop defined later)
- Fix: Use the correct variable that's in scope (e.g., `{x.y}` instead of undefined `{y}`)

### Matplotlib Animation Best Practices
- **Use `fig.canvas.new_timer()`** for non-blocking animations - DO NOT use deprecated APIs like `start_event_loop()`
- **Stop existing timers** before starting new ones to prevent timer leaks:
  ```python
  if timer is not None and timer.event_source is not None:
      timer.event_source.stop()
  ```
- **Timer callback pattern** for continuous playback:
  ```python
  def auto_play_step():
      if should_continue:
          do_one_step()
          if still_running:
              schedule_next_step()  # Recursive timer scheduling
  ```
- **Event-driven callbacks** keep GUI responsive vs blocking recursive calls

### Demo File Structure
Interactive demos should include:
- Step-by-step controls (Step Forward button)
- Auto-play with adjustable speed (Button + Slider)
- Reset functionality
- Visual panels for statistics and step-by-step explanations
- Proper resource cleanup on reset

### Testing Demos
- Test demo launch with timeout for GUI applications: `timeout 5 python demo.py`
- Verify both manual and auto-play modes work
- Check timer cleanup on reset prevents resource leaks