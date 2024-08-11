% Define system parameters
m = 0.111;   % Mass of the ball (kg)
R = 0.015;   % Radius of the ball (m)
g = -9.8;    % Acceleration due to gravity (m/s^2)
L = 1.0;     % Length of the beam (m)
d = 0.03;    % Distance from pivot to motor (m)
J = 9.99e-6; % Moment of inertia of the ball (kg*m^2)
s = tf('s'); % Define the Laplace variable
P_ball = -m*g*d/L/(J/R^2+m)/s^2; % Transfer function of the plant

% 1. Proportional Control (Initial)
Kp = 1;     % Proportional gain
C = pid(Kp); % Define the proportional controller
sys_cl = feedback(C*P_ball, 1); % Closed-loop system

% Step response for Proportional Control (Initial)
figure;
step(0.25*sys_cl)
title('Proportional Control (Initial)');
axis([0 70 0 0.5]) % Set axis limits

% 2. Proportional-Derivative Control (Initial Tuning)
Kp = 10;    % Proportional gain
Kd = 10;    % Derivative gain
C = pid(Kp, 0, Kd); % Define the PD controller
sys_cl = feedback(C*P_ball, 1); % Closed-loop system

% Step response for Proportional-Derivative Control (Initial Tuning)
figure;
t = 0:0.01:5;
step(0.25*sys_cl)
title('Proportional-Derivative Control (Initial Tuning)');

% 3. Proportional-Derivative Control (Refined Tuning)
Kp = 10;    % Proportional gain
Kd = 20;    % Refined Derivative gain
C = pid(Kp, 0, Kd); % Redefine the PD controller
sys_cl = feedback(C*P_ball, 1); % Closed-loop system

% Step response for Proportional-Derivative Control (Refined Tuning)
figure;
step(0.25*sys_cl)
title('Proportional-Derivative Control (Refined Tuning)');

% 4. Proportional-Derivative Control (Final Tuning)
Kp = 15;    % Final Proportional gain
Kd = 40;    % Final Derivative gain
C = pid(Kp, 0, Kd); % Redefine the PD controller
sys_cl = feedback(C*P_ball, 1); % Closed-loop system

% Step response for Proportional-Derivative Control (Final Tuning)
figure;
step(0.25*sys_cl)
title('Proportional-Derivative Control (Final Tuning)');
