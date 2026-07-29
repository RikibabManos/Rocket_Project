# 6-DOF Flight Dynamics & Control Simulator
## Overview
A high-fidelity, 6-Degrees-of-Freedom (6-DOF) launch vehicle simulator written and visualized in Python. This project models the ascent profile of a single-stage rocket, integrating rigid-body rotational kinematics, shifting centres of mass, and active Thrust Vector Control (TVC) to stabilize the vehicle against extreme aerodynamic disturbances. The example given follows data from the Saturn V spacecraft, imagined as a single stage
## The Core Physics & Mathematical Framework
This simulation does not rely on simplified 2D physics or pre-baked trajectories. Every microsecond of flight is calculated dynamically using a custom physics engine:
  - The Math (RK4 Integration): The core engine solves a 14-variable state vector $\mathbf{Y} = [x, y, z, v_x, v_y, v_z, q_0, q_1, q_2, q_3, \omega_x, \omega_y, \omega_z, m]$ using a 4th-Order Runge-Kutta (RK4) numerical integrator.
  - The Geometry (Quaternions over Euler Angles): Rotational kinematics are handled entirely via quaternions to absolutely eliminate Gimbal Lock during vertical ascent. The Direction Cosine Matrix (DCM) is used to translate forces between the rocket's localized Body Frame and the global coordinate system.
  - The Environment: Translational motion is integrated within the Earth-Centred Inertial (ECI) frame. The environment features a radial, altitude-dependent gravity model $$g(h)$$ and a localized atmospheric model to calculate dynamic pressure, local speed of sound, and Mach number. Aerodynamic coefficients ($C_D, C_N$, Centre of Pressure) are pulled dynamically via 2D Bilinear Interpolation from Saturn V wind-tunnel lookup tables.
  
### Visual Proof of Control: Significant Wind Shear Event

![Mission Control Telemetry](windshear_stabilization.gif)

The Edge Case: A physics engine is only as good as its control system. The telemetry dashboard above demonstrates the flight computer actively fighting a chaotic disturbance. Ten seconds after launch, the simulation injects a ~200 m/s lateral wind-shear event (190m/s and -80m/s in the x and y directions respectively).

You can watch the PID controller instantly calculate the pitch/yaw error and command the engine nozzle to gimbal, generating the exact restorative torque needed to prevent the rocket from tumbling and keep the ascent vector near-perfectly radial to the Earth.

## Interpretation of Dashboard
The Python-based Mission Control dashboard reads the telemetry data and provides real-time situational awareness of the vehicle:

-Left Panel (3D Trajectory): Displays the Earth (scaled as an oblate spheroid) and the ECI trajectory of the vehicle.

-Middle (Kinematics): Live readouts of True Altitude (Top) and Mach Number (Bottom).

-Top Right (Attitude Indicator): A localized ENU (East-North-Up) frame showing exactly how the nose of the rocket is oriented relative to "straight up."

-Bottom Right (TVC Gimbal Angle): A live plot of the engine's pitch and yaw gimbal angles. Spikes in this graph indicate the PID controller actively fighting aerodynamic flipping torques.

-IDE Terminal: A live feed of quantities crucial to track (Time, Fuel Mass, Total Mass and Altitude)

## Usage of Code
**(Important to run code correctly!)**

-Make sure all .py files are in the same folder

-Replace the two .cvs file paths on lines 24 and 37 of State_variable.py to match those on your device

-Alter constants in the respective sections of State_variable.py and visual_display.py to fit your needs

## Future Scope
### 1. Multi-Stage Orbital Dynamics
The current physics engine accurately simulates a single-stage ascent under extreme aerodynamic stress (modeling the Saturn V S-IC stage). The immediate next step is to upgrade the RK4 integrator to handle discrete staging events. This involves modeling dynamic mass-shedding, recalculating sudden shifts in the vehicle's Center of Gravity (CG) and moment of inertia tensor, and implementing time-delayed ignition sequences for upper-stage engines to achieve full orbital insertion.

### 2. Advanced Closed-Loop Guidance Algorithms
Currently, the vehicle relies on a tuned PID controller strictly for attitude stabilization (maintaining a radial ascent and fighting wind-shear). Future scope includes bridging this stabilization system with advanced closed-loop orbital guidance algorithms—such as Powered Explicit Guidance (PEG) or Linear Tangent Guidance. This will allow the virtual flight computer to dynamically calculate and steer toward fuel-optimal trajectories for Low Earth Orbit (LEO) under changing atmospheric conditions.

### 3. High-Performance C++ Physics Engine (SITL)
Currently, the entire 6-DOF simulation—including the RK4 numerical integrator, lookup tables, and the live telemetry dashboard—runs natively in Python. To achieve true faster-than-real-time execution under heavy computational loads, the next step is to decouple the physics from the visualization. The core flight dynamics engine and quaternion kinematics will be refactored into highly optimized C++. The existing Python architecture will be retained strictly for the UI (Mission Control dashboard), listening to the C++ backend via live UDP network sockets to establish an industry-standard Software-in-the-Loop (SITL) testing environment.

## Author
**Babikir Osman**
* [GitHub](https://github.com/RikibabManos)
* [LinkedIn](https://www.linkedin.com/in/babikir-osman-8a4261282)
