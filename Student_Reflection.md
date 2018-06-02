
### Reflection

#### The Model
*Student describes their model in detail. This includes the state, actuators and update equations.*

I used a similar simple kinematic model as in the MPC quiz. State vector contains `{x, y, psi, v, cte, epsi}` and actuators contains `delta, a`. The update equations are the same:
```c++
//x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
//y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
//psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
//v_[t+1] = v[t] + a[t] * dt
//cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
//epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
where
```c++
f = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
```
and `psides0 = atan(f'(x))`
```c++
psides0 = CppAD::atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3] * pow(x0, 2)));
```

#### Timestep Length and Elapsed Duration (N & dt)
*Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.*

I started with the value `N = 10` and `dt = 0.1`. I run the simulator and the green predicted path seems too long and it's not necessary and contains not useful late future information, so I changed to `N = 7`. Then I tried to increase the `ref_v` to make the car drive faster. So I need to increase the precision of my prediction. I decrease `dt` to `0.05`.  Then I set `N = 15` correspondingly so that the length of my predicted path stays the same.

#### Polynomial Fitting and MPC Preprocessing
*A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.*

Since the server returns waypoints using the map's coordinate system, I transformed them into the car's coordinate system. 

Transformation:
```c++
car_x[i] =  (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
car_y[i] = -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi);
```
Everything follows were done in the car's coordinate system, including displaying waypoints and prediction points, and the calculation of CTE and Epsi values.

Then I chose polynomials of degree 3 to fit the waypoints.
```c++
auto coeffs = polyfit(car_x, car_y, 3);
```

#### Model Predictive Control with Latency
*The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.*

I set latency as the given value 100ms `const double latency = 0.1`. Then I calulated the delayed state values in current state and actuators.
```c++
const double Lf = 2.67;
double steering_angle = j[1]["steering_angle"];
double x_late = v * latency;
double y_late = 0;
double psi_late = -v/Lf * steering_angle * latency;
// Delayed state vector after latency
Eigen::VectorXd state(6);
state << x_late, y_late, psi_late, v, cte, epsi;
```

#### Cost Function and Final Performance
I add coefficients for each considered attributes in the cost function:

```c++
const double coeff_cte = 10;
const double coeff_epsi = 10;
const double coeff_v = 1;
const double coeff_delta = 1000;
const double coeff_a = 1;
const double coeff_delta_d = 100000;
const double coeff_a_d = 1;
```

I tuned these parameters for a while. The coeffficients for `delta` and `delta_d` had to be set relatively large so that the car can turn smoothly. In the end my car can drive at a speed of `90 MPH` but may touch the curb once in one loop. It can easily drive without touching the curb at lower speed.
