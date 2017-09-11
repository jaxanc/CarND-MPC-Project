# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

### The Model

**Student describes their model in detail. This includes the state, actuators and update equations.**

The model implemented in this project is the kinematic model taught during class. The equations are summarized below.

```C++
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt;
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt;
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt;
v[t+1] = v[t] + a[t] * dt;
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt;
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt;
```

Where ```delta``` is the steering and ```a``` is the acceleration or throttle. The above equations are the update equations to calculate the next state.

### Timestep Length and Elapsed Duration (N & dt)

**Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.**

The final values are ```N = 10``` and ```dt = 0.05```. The timestep length ```N``` determines the target where the trajectory is heading to. If N is too small, the controller rapidly oscillates around the center and if N is too large, the controller can calculate the trajectory too large that's not enough to make a sharp turn. Where ```dt``` the time step determines how fast the controller can reach. ```dt``` of 0.05 produce the smoothest result. I have tried a range of different values from ```N = 5, 10, 15, 20``` and ```dt = 0.05, 0.1, 0.2```. The ratio between the parameters is also important and from experimenting with them the ```10:0.05``` is ideal.

### Polynomial Fitting and MPC Preprocessing

**A polynomial is fitted to waypoints.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.**

The waypoints are transformed into vehicle's perspective with the equation below before applying the polynomial fit. This simplifies the calculation as after the transform, the origin is at (0,0).

```C++
double dx = ptsx[i] - px;
double dy = ptsy[i] - py;
waypointsX[i] = dx * cos(-psi) - dy * sin(-psi);
waypointsY[i] = dx * sin(-psi) + dy * cos(-psi);
```

### Model Predictive Control with Latency

**The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.**

The method to correct for latency follows the discussion in this thread [How to incorporate latency into the model](https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391). The final implementation is:

```C++
// predict state in 100ms
double latency = 0.1;
psi += -v*delta/Lf*latency/2.67;
v += acceleration*latency;
```

If ```x``` and ```y``` are not adjusted with the same update function above it produces incorrect result and can be visualized with waypoints sliding off the road.
