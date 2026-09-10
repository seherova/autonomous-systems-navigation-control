# Autonomous Systems Algorithms & Simulations

A collection of Python implementations and simulations covering fundamental concepts in **autonomous systems**, including state estimation, navigation, radar tracking, sensor fusion, coordinate transformations, feedback control, path planning, and SLAM.

The repository follows a progressive approach: starting from simple state estimation with an Alpha-Beta filter and gradually moving toward Extended Kalman Filters, sensor fusion, autonomous navigation, control, planning, and simultaneous localization and mapping.

---

## Overview

Autonomous systems must continuously answer several fundamental questions:

* **Where am I?** → Localization & State Estimation
* **Where is the target?** → Tracking
* **How reliable are my sensors?** → Filtering & Sensor Fusion
* **How do different coordinate frames relate?** → Coordinate Transformations
* **How should I move toward a desired state?** → Feedback Control
* **Which route should I take?** → Path Planning
* **Can I localize while building a map?** → SLAM

The implementations in this repository explore these problems through mathematical models, noisy sensor simulations, estimation algorithms, and visualizations.

---

# Repository Contents

| #  | Topic                                | Main Concepts                                              |
| -- | ------------------------------------ | ---------------------------------------------------------- |
| 01 | Alpha-Beta Filter                    | State estimation, prediction, residual, correction         |
| 02 | 2D Navigation & Alpha-Beta Filtering | Heading, velocity decomposition, 2D tracking               |
| 03 | Radar Target Tracking                | Radar simulation, Alpha-Beta tracking, KF, EKF, RMSE       |
| 04 | GPS/INS Sensor Fusion                | Dead reckoning, EKF localization, GPS-denied navigation    |
| 05 | Sensor-to-Body Transformations       | Coordinate frames, rotation matrices, translation          |
| 06 | PID Control                          | P, PI, PD, PID, feedback control, Ziegler-Nichols tuning   |
| 07 | A* Path Planning                     | Graph search, heuristics, obstacle avoidance               |
| 08 | 1D EKF-SLAM                          | Localization, landmark estimation, covariance, uncertainty |

---

# 01 — Alpha-Beta Filter

**File:** `01_adım_adım_alfa_beta_filtresi.py`

The first implementation introduces state estimation through a simple **1D Alpha-Beta Filter**.

A moving object's position and velocity are estimated using the classical:

**Predict → Measure → Residual → Update**

cycle.

The state consists of position and velocity:

$$
X =
\begin{bmatrix}
x \\
v
\end{bmatrix}
$$

with the constant-velocity prediction model:

$$
x_{pred} = x + v\Delta t
$$

$$
v_{pred} = v
$$

The measurement residual is:

$$
r = z - x_{pred}
$$

and the state is corrected using:

$$
x = x_{pred} + \alpha r
$$

$$
v = v_{pred} + \frac{\beta}{\Delta t}r
$$

### Experiment

A vehicle moving at a constant velocity of **5 m/s** is observed using noisy position measurements.

Multiple Alpha-Beta parameter combinations are compared to demonstrate the trade-off between:

* fast response,
* noise rejection,
* and tracking delay.

This provides an intuitive introduction to the prediction/update structure later used by Kalman-based estimators.

---

# 02 — 2D Navigation & Alpha-Beta Filtering

**File:** `02_navigasyon_ve_alfabeta_filtresi.py`

The state estimation problem is extended from one dimension to a **2D navigation frame**.

The implementation first introduces the difference between mathematical angles and navigation heading conventions.

For a vehicle traveling with speed \(v\) and heading \(\psi\):

$$
v_x = v\sin(\psi)
$$

$$
v_y = v\cos(\psi)
$$

The state becomes:

$$
X =
\begin{bmatrix}
x \\
y \\
v_x \\
v_y
\end{bmatrix}
$$

A reusable `AlphaBetaFilter2D` class performs prediction and correction independently on both axes.

### Simulation

A vehicle travels northeast with:

* Heading: **45°**
* Speed: **10 m/s**

Gaussian noise is added to the simulated GPS-like position measurements.

The visualization compares:

**Ground Truth → Noisy Measurements → Filtered Trajectory**

and demonstrates how state estimation can recover a smoother vehicle trajectory from imperfect measurements.

---

# 03 — Radar Target Tracking

**File:** `03_radar_i̇z_takibi.py`

This section introduces a simulated **2D radar tracking system**.

Unlike the previous examples, the sensor naturally operates in polar coordinates:

$$
z =
\begin{bmatrix}
r \\
\theta
\end{bmatrix}
$$

where \(r\) represents range and \(\theta\) represents azimuth.

The radar model introduces Gaussian noise independently into range and angular measurements.

Polar observations can be transformed into Cartesian coordinates:

$$
x = r\cos(\theta)
$$

$$
y = r\sin(\theta)
$$

## Alpha-Beta Radar Tracker

A moving target is first tracked using a 2D Alpha-Beta tracker.

The tracker maintains:

$$
[x,\ y,\ v_x,\ v_y]
$$

and recursively estimates the target trajectory from noisy radar plots.

## From Alpha-Beta to Kalman Filtering

The implementation then progresses to Kalman-based estimation.

Instead of manually selecting fixed \(\alpha\) and \(\beta\) values, the Kalman Filter calculates a state-dependent correction using the **Kalman Gain**.

## Linear KF vs Extended Kalman Filter

The radar measurement function is nonlinear:

$$
r = \sqrt{x^2+y^2}
$$

$$
\theta = \mathrm{atan2}(y,x)
$$

An **Extended Kalman Filter (EKF)** is therefore implemented using a measurement Jacobian.

The repository compares:

**Linear Kalman Filter**

vs.

**Extended Kalman Filter**

using identical noisy radar measurements.

Their position estimation performance is evaluated quantitatively using **RMSE (Root Mean Squared Error)**.

This experiment demonstrates why nonlinear sensor models require more careful uncertainty propagation.

---

# 04 — GPS / INS Sensor Fusion & Localization

**File:** `04_gps_ins_füzyonu_lokalizasyon.py`

This implementation focuses on localization using two sensors with complementary characteristics.

### INS

Provides high-frequency motion information but accumulates errors due to:

* accelerometer bias,
* gyroscope bias,
* measurement noise,
* and dead-reckoning drift.

### GPS

Provides absolute position measurements at a lower frequency.

GPS does not accumulate dead-reckoning error but introduces measurement noise.

## Extended Kalman Filter Fusion

The EKF state is:

$$
X =
\begin{bmatrix}
x \\
y \\
v \\
yaw
\end{bmatrix}
$$

INS measurements are used during the high-frequency **prediction step**, while GPS measurements periodically perform the **correction step**.

The simulation compares:

**Ground Truth**

vs.

**INS-only Dead Reckoning**

vs.

**GPS Measurements**

vs.

**INS + GPS EKF Fusion**

---

## GPS-Denied Environment

A second simulation introduces a temporary GPS outage representing scenarios such as a tunnel.

During the outage:

* GPS updates stop,
* the EKF relies only on INS propagation,
* localization uncertainty increases,
* and dead-reckoning error begins accumulating.

Once GPS becomes available again, the EKF uses the new measurement innovation to correct the accumulated localization error.

This demonstrates one of the most important practical advantages of multi-sensor state estimation.

---

# 05 — Sensor & Platform Body Coordinate Frames

**File:** `05_sensör_ve_platform_body_eksenleri.py`

Sensors mounted on autonomous platforms rarely share exactly the same position and orientation as the vehicle body frame.

This implementation transforms measurements from a sensor-local coordinate system \(S\) into the platform body coordinate system \(B\).

The sensor observation is first represented in Cartesian form:

$$
P^S =
\begin{bmatrix}
r\cos(\theta) \\
r\sin(\theta) \\
0
\end{bmatrix}
$$

A rotation matrix represents the sensor mounting orientation:

$$
R_B^S =
\begin{bmatrix}
\cos(\alpha) & -\sin(\alpha) & 0 \\
\sin(\alpha) & \cos(\alpha) & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

and the sensor mounting position is represented by the translation vector:

$$
T_B^S =
\begin{bmatrix}
x_{off} \\
y_{off} \\
z_{off}
\end{bmatrix}
$$

The final transformation is:

$$
P^B = R_B^S P^S + T_B^S
$$

The implementation visualizes the:

* platform body axes,
* sensor position,
* mounting orientation,
* raw sensor observation,
* and transformed target position.

This concept forms the basis of multi-sensor calibration and sensor fusion on real robotic platforms.

---

# 06 — PID Feedback Control

**File:** `06_pid_kontrol.py`

State estimation determines what the system is doing.

Control determines **what the system should do next**.

This implementation introduces the classical PID feedback controller:

$$
u(t) =
K_p e(t)
+
K_i \int e(t)\,dt
+
K_d \frac{de(t)}{dt}
$$

where:

* \(K_p\) reacts to the current error,
* \(K_i\) accumulates past error,
* \(K_d\) reacts to the rate of change of the error.

## Controller Comparison

A simulated vehicle starts at position **0 m** and attempts to reach a target position of **100 m**.

Four controllers are compared:

**P → PI → PD → PID**

The physical model also includes mass and damping, allowing characteristics such as:

* steady-state error,
* overshoot,
* oscillation,
* response speed,
* and damping

to be observed directly.

## Ziegler-Nichols Tuning

The implementation also explores automatic controller tuning using the **Ziegler-Nichols closed-loop method**.

The critical gain \(K_u\) and critical oscillation period \(T_u\) are estimated and then used to derive P, PI, and PID gains.

This extends the experiment from manually selected controller parameters toward systematic controller tuning.

---

# 07 — A* Path Planning

**File:** `07_a_star_algoritması.py`

This implementation moves from estimation and control to **motion planning**.

The A* algorithm searches for an efficient path between a start and goal position using:

$$
f(n) = g(n) + h(n)
$$

where:

* \(g(n)\) is the actual accumulated path cost,
* \(h(n)\) is the estimated remaining cost to the goal.

The implementation uses **Euclidean distance** as the heuristic.

## Grid-Based Navigation

The planner operates on a 2D occupancy grid containing obstacles.

Eight possible movements are supported:

* horizontal,
* vertical,
* and diagonal movement.

Diagonal steps use a larger movement cost than horizontal and vertical steps.

## Search Visualization

The live visualization displays:

* explored nodes,
* candidate nodes,
* the current search path,
* start and goal positions,
* and the final optimal route.

## Random Solvable Maps

The implementation is extended to generate random obstacle maps.

Before running the main A* search, the environment is checked for connectivity.

If no valid route exists, a new random map is automatically generated until a solvable environment is obtained.

This makes each execution a different path-planning problem.

---

# 08 — 1D EKF-SLAM

**File:** `08_1d_ekf_slam.py`

The final implementation combines **localization and mapping** through a simplified 1D EKF-SLAM system.

The goal is to estimate both:

1. the robot position,
2. the positions of unknown landmarks.

For \(N\) landmarks, the state is represented as:

$$
X =
\begin{bmatrix}
x_r &
m_1 &
m_2 &
\cdots &
m_N
\end{bmatrix}^T
$$

Unlike independent estimators, SLAM maintains the correlations between the robot and landmarks through the covariance matrix \(P\).

## Prediction

When the robot moves:

$$
x_r \leftarrow x_r + u
$$

motion uncertainty is added to the robot state covariance.

## Measurement Update

For a landmark measurement:

$$
h(x) = m_i - x_r
$$

and the innovation is:

$$
y = z - h(x)
$$

The corresponding measurement Jacobian contains:

$$
H =
\begin{bmatrix}
-1 & \cdots & 1 & \cdots
\end{bmatrix}
$$

The EKF then calculates the Kalman Gain and jointly updates the robot and map estimates.

## Landmark Initialization

Unknown landmarks begin with very large uncertainty.

When a landmark is observed for the first time, its location is initialized from the current robot state and measured range.

Further observations reduce uncertainty and create correlations between the robot and landmark states.

## Simulation

The example environment contains three landmarks located at:

$$
5\,m,\quad12\,m,\quad18\,m
$$

The visualization tracks both estimated states and their uncertainty over time.

It demonstrates three important SLAM behaviors:

**Landmark initialization → uncertainty reduction → dead-reckoning uncertainty growth**

---

# Concept Progression

```text
Noisy Sensor Measurements
          │
          ▼
   Alpha-Beta Filter
          │
          ▼
    2D Navigation
          │
          ▼
    Radar Tracking
          │
          ▼
   Kalman Filter / EKF
          │
          ▼
   GPS + INS Fusion
          │
          ▼
 Coordinate Transformations
          │
          ├───────────────┐
          ▼               ▼
     PID Control     A* Path Planning
          │               │
          └───────┬───────┘
                  ▼
              EKF-SLAM
```

Together, these implementations represent several of the core building blocks required by an autonomous system:

**Perception → Estimation → Localization → Control → Planning → Mapping**

---

# Technologies

![Python](https://img.shields.io/badge/Python-3.x-blue?logo=python)
![NumPy](https://img.shields.io/badge/NumPy-Numerical%20Computing-blue)
![Matplotlib](https://img.shields.io/badge/Matplotlib-Visualization-orange)
![Jupyter](https://img.shields.io/badge/Jupyter-Compatible-orange?logo=jupyter)

Main Python libraries:

```text
numpy
matplotlib
ipython
```

The standard-library modules `heapq` and `time` are also used in some simulations.

---

# Installation

Clone the repository:

```bash
git clone https://github.com/YOUR_USERNAME/YOUR_REPOSITORY.git
cd YOUR_REPOSITORY
```

Install the required Python packages:

```bash
pip install numpy matplotlib ipython
```

For the live visualization examples, running the scripts inside **Jupyter Notebook or Google Colab** provides the intended interactive behavior.

---

# Running an Example

For example:

```bash
python "01_adım_adım_alfa_beta_filtresi.py"
```

or:

```bash
python "07_a_star_algoritması.py"
```

Individual scripts are independent and can be explored separately.

---

# Key Topics

`State Estimation` · `Alpha-Beta Filter` · `Kalman Filter` · `Extended Kalman Filter` · `Sensor Fusion` · `Radar Tracking` · `Localization` · `INS` · `GPS` · `Coordinate Frames` · `PID Control` · `A*` · `Path Planning` · `SLAM` · `Autonomous Systems`

---

# Educational Scope

The implementations are intended to make the mathematical foundations of autonomous systems easier to understand through simulation and visualization.

They focus on algorithmic intuition and learning rather than production-level robotics or safety-critical deployment.

---

## License

Add a license according to how you want the repository to be used and shared.
