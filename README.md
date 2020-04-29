# State Estimation

## Scenario 1 - Noisy GPS and Accelerometer Data
I loaded the gps and accelerometer logs into a Python Notebook here: [https://app.mode.com/arush/reports/c676b4cb074b](https://app.mode.com/arush/reports/c676b4cb074b)

```
import statistics
import numpy as np

gps = np.array(datasets[1].quad_gps_x)
accel = np.array(datasets[0].quad_imu_ax)

accel_s = statistics.pstdev(accel)
gps_s = statistics.pstdev(gps)
print(accel_s)
print(gps_s)
```

#### Output

```
Accelerometer.X Sigma: 0.5118249416490985
GPS.X Sigma: 0.6866827234046664
```