# CarND Controls MPC Project
Self-Driving Car Engineer Nanodegree Program

## Discussion of Rubric points

1. Student describes their model in detail. This includes the state, actuators and update equations.
    - There are two types of vehicle models. 
        1. Dynamic model:
            This is aim to embody the actual vehicle dynamics as closely as possible
        2. Kinematic model:
            This is simplification of dynamic models that ignore tire forces, gravity & masses
            Simplification reduces the accuracy but at low/moderate speed they are having accuracy close to dynamic models

    - For this project we are using Kinematic model. 

        The model's state has 6 different values: 
        * px: position in X
        * py: position in Y
        * psi: car's heading direction
        * v: velocity 
        * cte: cross track error
        * epsi: error psi  
        
        The model has also 2 actuators :
        * steering angle (steer_value)
        * throttle (throttle_value)  

        Following are the Model update equations used:
        * x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
        * y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
        * psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
        * v[t+1] = v[t] + a[t] * dt
        * cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
        * epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
        where,
               Lf - the distance between the center of mass of the vehicle and the front wheels.   

2. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
`T = N * dt`
Where, T : prediction horizon which is duration over which future predictions are made
       N : No. of timestep in horizon
       dt : time elapse between actuations

       Experiment:
       Changing dt(0.3 <= dt <= 0.01> >) keeping keeping N (N=25) constant :
       a. N = 25, dt = 0.3 -->  Results are not stable. Both trajectories goes out of the road
       b. N = 25, dt = 0.09 --> Results are improved but still oscillating, making vehicle go out of the road
       c. N = 25, dt = 0.03 --> Results are stable. Vehicle is able to complete the track.
       d. N = 25, dt = 0.01 --> Vehicle is able to complete the track but it is dancing :)

     After above experiment, selected N=25, dt=0.03

3. A polynomial is fitted to waypoints.If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The waypoints of the car reference trajectory are given in MAP co-ordinate system. We need to convert them into vehicle co-ordinate system by translation and rotation.
ptsx[i] , ptsy[i] below are the new waypoint coordinates in vehicle coordinate system. 

Once we have waypoints are in vehicle co-ordinate system, a third order polynomial is then fitted. Below is the code snippet from main.cpp

```
// MPC Preprocessing
for (unsigned int i = 0; i < ptsx.size(); i++)
    {
        // Shift Car reference Angle to 90 degrees
        double shift_x = ptsx[i] - px;
        double shift_y = ptsy[i] - py;

        ptsx[i] = shift_x * cos(0 - psi) - shift_y * sin(0 - psi);
        ptsy[i] = shift_x * sin(0 - psi) + shift_y * cos(0 - psi);
    }

//Polynomial Fitting

Eigen::VectorXd ptsx_t = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
Eigen::VectorXd ptsy_t = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());

// Measure Polynomial Coefficients.
auto coeffs = polyfit(ptsx_t, ptsy_t, 3);
```


4. The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Additional latency of 100ms is artificially added before sending actuations to the simulator. If we wouldn't handle the latency problem in our controller then oscilations and generally poor trajectories can occur. 

Steps used:

- Predict the state forward at the latency time before feeding into the solver
```
// Predict the state
          const double t_delta = 0.1;
          px = px + v * cos(psi) * t_delta;
          py = py + v * sin(psi) * t_delta;
          psi = psi + (v * steer_value * -t_delta) / Lf ;
          v = v + throttle_value * t_delta;
```

- Map the waypoints from global coordinates to new_vehicle coordinates as described in last question
- Then we calulate cte & epsi at the beginning of timestep & update them by incorporating the effect of latency
```
      double cte = polyeval(coeffs, 0);
          //double epsi = psi - atan(coeffs[1]);
          double epsi = -atan(coeffs[1]);    // Since psi becomes zero we can remove that from the equation.

          // Incorporate latency considerations
          cte = cte + v * sin(epsi) * t_delta;
          epsi = epsi + v * steer_value/Lf * t_delta;
```
- Feed the state values and coeffs to the solver.
```
          // Define the state vector
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          // Call MPC Solver
          auto vars = mpc.Solve(state, coeffs);

```

   