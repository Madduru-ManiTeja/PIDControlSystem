% Define the PID controller parameters
Kp = 1;   % Proportional gain
Ki = 1;   % Integral gain
Kd = 1;   % Derivative gain

% Create a transfer function for the PID controller
s = tf('s');
C = Kp + Ki/s + Kd*s;  % PID controller transfer function

% Define the plant transfer function for a mass-spring-damper system
P = 1/(s^2 + 10*s + 20);

% Plot the open-loop step response of the plant
figure(1)
step(P);  % Open-loop response of the plant

% Proportional control only (P controller)
Kp = 300;  % Increase the proportional gain
C = pid(Kp);  % Create a P controller with only proportional gain
T = feedback(C*P,1);  % Closed-loop transfer function with unity feedback

% Plot the step response of the system with P control
t = 0:0.01:2;
figure(2)
step(T,t);  % Step response of the system with P control

% Proportional-Derivative control (PD controller)
Kp = 300;  % Proportional gain
Kd = 10;   % Derivative gain
C = pid(Kp,0,Kd);  % Create a PD controller with proportional and derivative gains
T = feedback(C*P,1);  % Closed-loop transfer function with unity feedback

% Plot the step response of the system with PD control
t = 0:0.01:2;
figure(3)
step(T,t);  % Step response of the system with PD control

% Proportional-Integral control (PI controller)
Kp = 30;   % Proportional gain
Ki = 70;   % Integral gain
C = pid(Kp,Ki);  % Create a PI controller with proportional and integral gains
T = feedback(C*P,1);  % Closed-loop transfer function with unity feedback

% Plot the step response of the system with PI control
t = 0:0.01:2;
figure(4)
step(T,t);  % Step response of the system with PI control

% Repeat the PI control with the same gains (unnecessary duplication)
Kp = 30;  % Proportional gain
Ki = 70;  % Integral gain
C = pid(Kp,Ki);  % Create a PI controller with the same gains as above
T = feedback(C*P,1);  % Closed-loop transfer function with unity feedback

% Plot the step response of the system with PI control (duplicate)
t = 0:0.01:2;
figure(5)
step(T,t);  % Step response of the system with PI control (duplicate)

% Automatically tune a PID controller using MATLAB's pidtune function
opts = pidtuneOptions('CrossoverFrequency',32,'PhaseMargin',90);  % Set tuning options
[C, info] = pidtune(P, 'pid', opts);  % Tune the PID controller

