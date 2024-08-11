% Define initial PID gains for demonstration
Kp = 1;  % Proportional gain
Ki = 1;  % Integral gain
Kd = 1;  % Derivative gain

% Create a transfer function 's' for the Laplace variable
s = tf('s');

% Define the PID controller using the gains Kp, Ki, and Kd
C = Kp + Ki/s + Kd*s;

% Alternatively, define the PID controller using MATLAB's pid object
C = pid(Kp, Ki, Kd);

% System parameters for the cruise control problem
m = 1000;  % Mass of the vehicle in kg
b = 50;    % Damping coefficient in N·s/m
r = 10;    % Reference speed in m/s

% Define the plant transfer function P_cruise based on the system model
P_cruise = 1/(m*s + b);

% Proportional Control Example
Kp = 100;  % Proportional gain
C = pid(Kp);  % Define a P controller

% Obtain the closed-loop transfer function T with unity feedback
T = feedback(C*P_cruise, 1);

% Time vector for simulation
t = 0:0.1:20;

% Plot the step response of the closed-loop system for P controller
figure(1)
step(r*T, t);
axis([0 20 0 10]);  % Set axis limits

% Increase the proportional gain to 5000 to reduce steady-state error
Kp = 5000;
C = pid(Kp);
T = feedback(C*P_cruise, 1);

% Plot the step response with higher proportional gain
figure(2)
step(r*T, t);
axis([0 20 0 10]);  % Set axis limits

% PI Control Example
Kp = 600;  % Proportional gain
Ki = 1;    % Integral gain
C = pid(Kp, Ki);  % Define a PI controller

% Obtain the closed-loop transfer function T with PI controller
T = feedback(C*P_cruise, 1);

% Plot the step response with PI control
figure(3)
step(r*T, t);
axis([0 20 0 10]);  % Set axis limits

% Adjust PI gains to achieve desired performance
Kp = 800;
Ki = 40;
C = pid(Kp, Ki);

% Obtain the closed-loop transfer function T with updated PI controller
T = feedback(C*P_cruise, 1);

% Plot the step response with updated PI gains
figure(4)
step(r*T, t);
axis([0 20 0 10]);  % Set axis limits

% PID Control Example
Kp = 1;  % Proportional gain
Ki = 1;  % Integral gain
Kd = 1;  % Derivative gain
C = pid(Kp, Ki, Kd);  % Define a PID controller

% Obtain the closed-loop transfer function T with PID controller
T = feedback(C*P_cruise, 1);

% Plot the step response with PID control
figure(5)
step(r*T, t);
axis([0 20 0 10]);  % Set axis limits
