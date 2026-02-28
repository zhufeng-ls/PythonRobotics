# Learning Model Predictive Control (MPC) with PythonRobotics

This tutorial guides you through the Model Predictive Control (MPC) implementation in the PythonRobotics repository.

## 1. What is MPC?
MPC is an advanced method of process control that is used to control a process while satisfying a set of constraints. It has been in use in the process industries in chemical plants and oil refineries since the 1980s. In recent years it has also been used in power system balancing models and in power electronics. Model predictive controllers rely on dynamic models of the process, most often linear empirical models obtained by system identification.

## 2. Key Files in PythonRobotics
The repository contains several MPC examples, but we will focus on the most practical one for path tracking:

- **`PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py`**
    - This is a Linear MPC implementation that iterates to approximate nonlinear dynamics.
    - It controls both **steering** and **speed**.

## 3. Mathematical Formulation

### State Vector
The state of the vehicle is defined as:
$$ z = [x, y, v, \phi] $$
Where:
- $x, y$: Position
- $v$: Velocity
- $\phi$: Yaw angle

### Control Inputs
The control inputs are:
$$ u = [a, \delta] $$
Where:
- $a$: Acceleration
- $\delta$: Steering angle

### Vehicle Model (Kinematics)
The vehicle is modeled as a kinematic bicycle model:
$$
\begin{aligned}
x_{k+1} &= x_k + v_k \cos(\phi_k) dt \\
y_{k+1} &= y_k + v_k \sin(\phi_k) dt \\
v_{k+1} &= v_k + a_k dt \\
\phi_{k+1} &= \phi_k + \frac{v_k}{L} \tan(\delta_k) dt
\end{aligned}
$$
*Note: In the code, this model is linearized around the operating point to use with a Convex QP solver.*

### Cost Function
subject to:
$$
\begin{aligned}
\min_{u} J = \sum_{k=0}^{T-1} & (z_k - z_{ref,k})^T Q (z_k - z_{ref,k}) + \\
& u_k^T R u_k + \\
& (u_k - u_{k-1})^T R_d (u_k - u_{k-1})
\end{aligned}
$$
This minimizes:
1.  **Tracking Error**: Difference between actual state and reference trajectory ($Q$).
2.  **Control Effort**: Magnitude of inputs ($R$).
3.  **Control Smoothness**: Change in inputs ($R_d$).

### Constraints
The optimization is subject to physical constraints:
- Max Speed: $v \in [v_{min}, v_{max}]$
- Max Steering: $\delta \in [-\delta_{max}, \delta_{max}]$
- Max Acceleration: $a \in [-a_{max}, a_{max}]$
- Max Steering Rate: $\dot{\delta} \in [-\dot{\delta}_{max}, \dot{\delta}_{max}]$

## 4. Code Walkthrough

### `linear_mpc_control` Function
This is the core function.
1.  **Define Variables**: `cvxpy.Variable` creates optimization variables for State ($NX \times (T+1)$) and Input ($NU \times T$).
2.  **Build Cost**: Loops through horizon `T`, adding quadratic costs (`cvxpy.quad_form`).
3.  **Build Constraints**:
    - **Dynamics**: `x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C` (Linearized model).
    - **Physical Limits**: Adds inequalities for speed, accel, steer.
4.  **Solve**: `prob.solve(solver=cvxpy.CLARABEL)` solves the Quadratic Program (QP).

### `iterative_linear_mpc_control` Function
Since the vehicle model is nonlinear, we can't just run Linear MPC once.
1.  **Predict Motion**: Uses the current control solution to predict the future trajectory using the *nonlinear* model.
2.  **Linearize**: Linearizes the model around this predicted trajectory.
3.  **Solve LMPC**: Solves the linear MPC problem.
4.  **Iterate**: Repeats this process (default 3 times) to converge to a better solution.

## 5. How to Run
To run the simulation, use the following command (make sure you are in the directory and `PYTHONPATH` includes the repo root):

```bash
export PYTHONPATH=$PYTHONPATH:/your/path/to/PythonRobotics
python3 PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py
```

This will open a matplotlib window showing the car tracking the path.

## 6. Next Steps
- Try changing `TARGET_SPEED` or `MAX_STEER` in the code to see how behavior changes.
- Explore `PathTracking/cgmres_nmpc` for a more advanced Nonlinear MPC approach that uses Continuation/GMRES instead of iterative linearization.
