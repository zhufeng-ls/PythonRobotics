---
description: PythonRobotics Project Core Rules
---

# 🛡️ PythonRobotics Core Protocols

## 1. Directory Permissions & Roles
*   **`PythonRobotics/` (Reference Library)**
    *   🛑 **READ-ONLY**: Do NOT modify original files.
    *   ✅ **Reference**: Use for reading logic and design patterns.
    *   🏃 **Execution**: Can be run directly (e.g., `python3 PathPlanning/AStar/a_star.py`).
*   **`PythonRoboticsLearning/` (Workspace)**
    *   ✅ **READ-WRITE**: All edits, experiments, and notes go here.
    *   📂 **Structure**:
        *   `notes/`: Markdown study notes.
        *   `experiments/`: Executable code (copies or imports).
        *   `code_snippets/`: Helper tools.

## 2. Visualization Standards
*   ⚠️ **ENGLISH ONLY**: All keys, labels, titles, and legends in plots must be English.
    *   ❌ `plt.title("路径规划")`
    *   ✅ `plt.title("Path Planning")`

## 3. Workflow Protocol (RAED)
1.  **Read**: Analyze original code in `PythonRobotics/`.
2.  **Analyze**: Understand the math/logic.
3.  **Experiment**: Copy to `PythonRoboticsLearning/experiments/` to modify.
4.  **Document**: Record findings in `PythonRoboticsLearning/notes/`.

## 4. Documentation
*   Do not create overhead documentation unless requested.
*   Keep comments clear in code.
