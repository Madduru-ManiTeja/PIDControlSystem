% Define motor parameters
J = 0.01;  % Rotational inertia (kg·m^2) - This represents the inertia of the motor's rotor.
b = 0.1;   % Viscous friction coefficient (N·m·s) - This represents the damping or friction opposing the motor's motion.
K = 0.01;  % Electromotive force constant (V·s/rad) - This is the motor constant relating the back EMF and torque.
R = 1;     % Electric resistance (Ω) - This represents the resistance in the motor windings.
L = 0.5;   % Electric inductance (H) - This represents the inductance of the motor windings.

% Define transfer function variable
s = tf('s');  % Create a Laplace variable 's' for defining transfer functions in the s-domain.

% Define motor transfer function
P_motor = K / ((J*s + b)*(L*s + R) + K^2);  
% This is the open-loop transfer function of the DC motor, derived from its dynamic equations.

% Proportional Control
Kp = 100;  % Proportional gain - controls the system response speed and steady-state error.
C = pid(Kp);  % Define a PID controller with only proportional gain (no integral or derivative terms).
sys_cl = feedback(C*P_motor,1);  % Close the loop by connecting the controller with the plant (motor).

% Plot step response for proportional control
t = 0:0.01:5;  % Time vector for the simulation (from 0 to 5 seconds with a step of 0.01 seconds).
figure(1);  % Create a new figure window for plotting.
step(sys_cl,t);  % Plot the step response of the closed-loop system.
grid;  % Add grid lines to the plot for better readability.
title('Step Response with Proportional Control');  % Add a title to the plot.
% Purpose: To observe the behavior of the system with only proportional control.

% PID Control with Small Ki and Small Kd
Kp = 75;  % Proportional gain
Ki = 1;   % Integral gain - helps eliminate steady-state error over time.
Kd = 1;   % Derivative gain - helps reduce overshoot and improve stability.
C = pid(Kp,Ki,Kd);  % Define a PID controller with the given gains.
sys_cl = feedback(C*P_motor,1);  % Close the loop with the PID controller.

% Plot step response for initial PID control
figure(2);  % Create a new figure window for plotting.
step(sys_cl,[0:1:200]);  % Plot the step response over a larger time span (0 to 200 seconds).
title('PID Control with Small Ki and Small Kd');  % Add a title to the plot.
% Purpose: To observe how a small integral gain affects the system's steady-state error.

% PID Control with Large Ki and Small Kd
Kp = 100;  % Proportional gain
Ki = 200;  % Increased integral gain to speed up the elimination of steady-state error.
Kd = 1;    % Small derivative gain to control overshoot.
C = pid(Kp,Ki,Kd);  % Define a PID controller with the new gains.
sys_cl = feedback(C*P_motor,1);  % Close the loop with the updated PID controller.

% Plot step response for adjusted Ki
figure(3);  % Create a new figure window for plotting.
step(sys_cl, 0:0.01:4);  % Plot the step response over a shorter time span (0 to 4 seconds).
grid;  % Add grid lines to the plot.
title('PID Control with Large Ki and Small Kd');  % Add a title to the plot.
% Purpose: To see the effect of a larger integral gain on settling time and overshoot.

% PID Control with Large Ki and Large Kd
Kp = 100;  % Proportional gain
Ki = 200;  % Integral gain
Kd = 10;   % Increased derivative gain to further reduce overshoot and improve stability.
C = pid(Kp,Ki,Kd);  % Define a PID controller with the final gains.
sys_cl = feedback(C*P_motor,1);  % Close the loop with the final PID controller.

% Plot step response for final PID control
figure(4);  % Create a new figure window for plotting.
step(sys_cl, 0:0.01:4);  % Plot the step response over a short time span (0 to 4 seconds).
grid;  % Add grid lines to the plot.
title('PID Control with Large Ki and Large Kd');  % Add a title to the plot.
