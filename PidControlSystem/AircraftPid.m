% Define the system
s = tf('s');
P_pitch = (1.151*s + 0.1774)/(s^3 + 0.739*s^2 + 0.921*s);

% Control System Designer for Proportional Control
controlSystemDesigner(P_pitch);

% Implementing PID Controller
C_pid = pidtune(P_pitch, 'PID');
T = feedback(C_pid * P_pitch, 1);

% Step Response
figure;
step(0.2*T);
title('Closed-Loop Step Response with PID Controller');
