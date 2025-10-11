*State Vector*
x = [h, v] Transpose.
h: True altitude above ground level (AGL) in meters.
v: Vertical speed in meters per second (m/s).

*assumption*: Constant velocity model. 

*State Prediction Model*
h_t=h_(t-1) + v_(t-1) * dt
v_t=v_(t-1)

In matrix:
x_t = [1 dt
       0 1] * x_(t-1) + w_t
where dt = 0.01s (10ms between readings).
F = [1 0.01
     0 1]
w_t: Process noise with covariance Q.

v0 = 0 - Initial vertical speed is 0 m/s.

GPS adjustment: GPS altitude is relative to mean sea level, not ground level. To estimate
AGL from GPS, we need to subtract the ground elevation. Assuming ground is 150m above sea level:
h_gps_agl = h_gps - 150


*Measurement Model*
Each sensor (GPS, altimeter 1, altimeter 2) measures height directly. The measurement model is:
z_t = [1 0] * x_t + v_t
where [1 0]: Maps the state (height and vertical speed) to the measurement (height ONLY).
Let's call H = [1 0]
z_t: The sensor measurement (e.g. GPS or altimeter reading).
v_t: Measurement noise with covariance R.


Covariance Matrices
State covariance P: Tracks uncertainty in state (h and v) - motion model.
Starts as P_0 = [[100 0]
                 [0   1]]
100 m^2 uncertainty in altitude, 1 (m/s)^2 in vertical speed.
THis one changes. it’s what the filter believes about its own uncertainty)

Process noice covariance Q - uncertainity in the sensor:
Q = [[0.0001   0]]
     [0       0.000001]]
sigma_h = 0.1m, sigma_v = 0.01 m/s.
small noise: 0.1m for height, 0.01m/s for velocity.
This one stays constant.

Measurement noise covariance R:
R_altimeter = 24 # Altimeter noise variance
R_gps = 100 (assume GPS AGL has +-10m m noise)

Handling Faulty Altimeter Readings
Outlier detection: Reject measurements where the residual (z_t - h_t) exceeds a threshold (e.g., 3 std deviation based on 
                    P and R).

Initial conditions:
Height h_0 = start with GPS AGL reading.
Vertical speed v_0 = 0 m/s.
F = [[1 0.01]
     [0 1]]
H = [1 0]
Prediction Step:
x_t = F * x_(t-1)
P_t = F * P_(t-1) * F^T + Q

Update Step (for each measurement):
z_t = [z_gps_agl, z_altimeter1, z_altimeter2]  # Available measurements
H = [[1 0], [1 0], [1 0]]  # Measurement matrix for height only
R = diag([R_gps, R_altimeter, R_altimeter])  # Measurement noise covariance
y_t = z_t - H * x_t  # Measurement residual
S_t = H * P_t * H^T + R  # Residual covariance
Outlier gate: d^2 = y_t^T * S_t^(-1) * y_t
If d^2 > threshold (e.g., 9 for 3 std deviations), reject
K_t = P_t * H^T * S_t^(-1)  # Kalman gain
x_t = x_t + K_t * y_t  # Updated state estimate
P_t = (I - K_t * H) * P_t  # Updated estimate covariance
where I is the identity matrix.

Output:
X_t[0] is the estimated height above ground level at time t.
(uncertainty in the estimate can be derived from P_t[0][0], the variance in height.)