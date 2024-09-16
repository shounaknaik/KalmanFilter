# Kalman Filter
In this repository, we have implemeted a Kalman filter to track the motion of a drone flying through open space. The drone's position is monitored by a Qualisys motion capture system, which uses infrared cameras to track spherical retroreflective markers on the drone. The system can capture the drone's motion at a rate of over 100 Hz.

## System Modelling
The system is modeled using a state vector consisting of the drone’s position and velocity. Let **p** = [x, y, z]^T denote the position vector, and **ṗ** = [ẋ, ẏ, ż]^T denote the velocity vector. With this representation, the state vector becomes:

x = [x, y, z, ẋ, ẏ, ż]^T

The noise is assumed to be zero. Control input vector u(t) is expressed as:
$$u(t) = m \ddot{p}$$  

In this model, we have the input vectors at each timestamp - $u$. Thus we estimate the accelaration from the $u$ because of our known model. Process Model is when we rely on a model. Then we correct our estimate using our sensors or Measurement model. This is a case of odometry based on physical tracking of the drone.

## Process Model 

The state transition model is given by (Standard Kalman filter equation)

$$
\dot{x} = A x + B u
$$
The state transition matrix **A** becomes as follows. This can be derived by merging earlier equations:

$$
A = \begin{bmatrix}
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$
The input matrix **B** becomes:

$$
B = \begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & 0\\
\frac{1}{m} & 0 & 0 \\
0 &\frac{1}{m} &0\\
0 & 0 & \frac{1}{m}
\end{bmatrix}
$$

## Continous to Discrete
For working on real data, we have to discretize the continous model to a discrete model. We use Euler one step method to discretize.
$$ x(t+\Delta t) = x(t) + \Delta t \cdot (A \cdot x(t) + B \cdot u(t) + N(t)) $$

The state transition matrix **F** is given by:
$$
F = I + \delta t A = \begin{bmatrix}
1 & 0 & 0 & \delta t & 0 & 0 \\
0 & 1 & 0 & 0 & \delta t & 0 \\
0 & 0 & 1 & 0 & 0 & \delta t \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$
The input matrix **G** is given by:

$$
G = \delta t B = \begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & 0\\
\frac{\Delta t}{m} & 0 & 0 \\
0 & \frac{\Delta t}{m} & 0\\
0 & 0 & \frac{\Delta t}{m}
\end{bmatrix}
$$


## Measurement Model

$$
y = C x + V(noise)
$$
Since the sensor here is the IMU, we have a simple observation model.
If the measurement is position (**z = p**), then:

$$
C_{\text{position}} = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0
\end{bmatrix}
$$

If the measurement is velocity (**$z = \dot{p}$**), then:

$$
C_{\text{velocity}} = \begin{bmatrix}
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$



Since the Kalman filter is a discrete filter, we first need to discretize the given system using the one-step Euler integration method. The first-order Markov assumption makes the Kalman filter system model a first-order differential equation:

$$
\dot{x} = f(x, u, N(0, Q)) = A x + B u + E N(0, Q)
$$


## Kalman Filter
### 1. P Matrix (Error Covariance Matrix)
- **Purpose**: Represents the covariance of the error in the estimated state.
- **Role**: It tracks the uncertainty of the state estimate. As the filter processes new measurements and predictions, the **P matrix** gets updated to reflect how much confidence the filter has in the current state estimate.
- **Dimension**: If the state vector $\mathbf{x}$ has $n$ dimensions, $P$ is an $n* n$ matrix.
- **Key Point**: The diagonal elements represent the variance (uncertainty) of each state variable, while the off-diagonal elements represent the correlations between state variables.

### 2. Q Matrix (Process Noise Covariance Matrix)
- **Purpose**: Models the covariance of the process noise (uncertainty in the system model or external disturbances).
- **Role**: The **Q matrix** represents the uncertainty in the system dynamics or process model. It affects how the filter predicts the next state in the absence of new measurements. The higher the values in **Q**, the more uncertainty the filter assumes in its model.
- **Dimension**: An $n * n$ matrix (where $n$ is the size of the state vector).
- **Key Point**: If the process model is perfect, \(Q\) would be zero. In practice, nonzero values reflect the imperfections or random disturbances in the system dynamics.

### 3. R Matrix (Measurement Noise Covariance Matrix)
- **Purpose**: Describes the covariance of the noise in the sensor measurements.
- **Role**: The **R matrix** accounts for how much noise is present in the sensor data. It helps determine how much weight the EKF should place on the sensor data during the update step (measurement correction). Higher values in **R** mean the filter will rely less on the noisy measurements.
- **Dimension**: If the measurement vector $\mathbf{z}$ has \(m\) dimensions, \(R\) is an \(m * m\) matrix.
- **Key Point**: This matrix represents the confidence in the sensor readings. A highly reliable sensor has small diagonal entries in **R**.

### 4. K Matrix (Kalman Gain Matrix)
- **Purpose**: Balances the trade-off between the model predictions and the sensor measurements.
- **Role**: The **K matrix** is calculated using the **P**, **R**, and **H** (measurement model) matrices. It determines how much of the correction step is influenced by the sensor measurements versus the predicted state. The Kalman Gain is not directly a covariance matrix, but it is computed from them.
- **Key Point**: The Kalman gain is used to update the state estimate by weighting the predicted state and measurement error appropriately.  

$V$ is considered to be identity in our case.


https://aleksandarhaber.com/kalman-filter/ -
Important resource for understanding the following equations.   
$ \bar{\mu}_t = F \mu_{t-1} + G u_t $

$ \bar{\Sigma}_t = F \Sigma_{t-1} F^T + V Q V^T $

$ K = \bar{\Sigma}_t C^T (C \bar{\Sigma}_t C^T + R)^{-1} $

$ \mu_t = \bar{\mu}_t + K(z_t - C \bar{\mu}_t) $

$ \Sigma_t = (I - K C) \bar{\Sigma}_t $  

Q matrix is initialized with the help of this [link](https://web.archive.org/web/20220202010928/https:/www.kalmanfilter.net/covextrap.html). This is a case of a model having control input thus 
$$ Q = \sigma^2 (G G^T)$$  
P matrix is initialized with a large number since we don't know the initial state perfectly. If known, can be set to 0.  
R matrix chosen according to the type of measurement series we have. $ R = \sigma*\sigma * I $. $\sigma$ ranges from 0.1 to 1.0 in our case.  

## Results

<figure>
  <img src="./Output.png" alt="Drone Track">
  <figcaption style="text-align: center;">Drone Tracking by Kalman Filter</figcaption>
</figure>


