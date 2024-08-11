% Parameters of the DC Motor
J = 3.2284E-6;  % Inertia of the motor
b = 3.5077E-6;  % Damping coefficient of the motor
K = 0.0274;     % Motor constant (torque constant)
R = 4;          % Electrical resistance of the motor windings
L = 2.75E-6;    % Electrical inductance of the motor windings

% Define the transfer function 's' for Laplace variable
s = tf('s');

% Transfer function of the DC motor
P_motor = K/(s*((J*s+b)*(L*s+R)+K^2));

% Proportional control: Varying the proportional gain Kp
Kp = 1;
for i = 1:3
    C(:,:,i) = pid(Kp);  % Create a PID controller with different Kp values
    Kp = Kp + 10;        % Increment Kp by 10 in each iteration
end
sys_cl = feedback(C*P_motor,1);  % Calculate the closed-loop system

% Plot the step response for different Kp values
figure;  % Create a new figure window
t = 0:0.001:0.2;  % Time vector for simulation
step(sys_cl(:,:,1), sys_cl(:,:,2), sys_cl(:,:,3), t)
ylabel('Position, \theta (radians)')
title('Response to a Step Reference with Different Values of K_p')
legend('Kp = 1',  'Kp = 11',  'Kp = 21')
% PI control: Adding integral action with a fixed Kp
Kp = 21;
Ki = 100;
for i = 1:5
    C(:,:,i) = pid(Kp,Ki);  % Create a PID controller with fixed Kp and varying Ki
    Ki = Ki + 200;          % Increment Ki by 200 in each iteration
end

sys_cl = feedback(C*P_motor,1);  % Calculate the closed-loop system

% Plot the step response for different Ki values
figure;  % Create a new figure window
t = 0:0.001:0.4;  % Time vector for simulation
step(sys_cl(:,:,1), sys_cl(:,:,2), sys_cl(:,:,3), t)
ylabel('Position, \theta (radians)')
title('Response to a Step Reference with K_p = 21 and Different Values of K_i')
legend('Ki = 100', 'Ki = 300', 'Ki = 500')

% PID control: Adding derivative action with fixed Kp and Ki
Kp = 21;
Ki = 500;
Kd = 0.05;

for i = 1:3
    C(:,:,i) = pid(Kp,Ki,Kd);  % Create a PID controller with fixed Kp, Ki, and varying Kd
    Kd = Kd + 0.1;             % Increment Kd by 0.1 in each iteration
end

sys_cl = feedback(C*P_motor,1);  % Calculate the closed-loop system

% Plot the step response for different Kd values
figure;  % Create a new figure window
t = 0:0.001:0.1;  % Time vector for simulation
step(sys_cl(:,:,1), sys_cl(:,:,2), sys_cl(:,:,3), t)
ylabel('Position, \theta (radians)')
title('Step Response with K_p = 21, K_i = 500 and Different Values of K_d')
legend('Kd = 0.05', 'Kd = 0.15', 'Kd = 0.25')

% Display performance metrics of the system with Kp = 21, Ki = 500, and Kd = 0.15
stepinfo(sys_cl(:,:,2))

