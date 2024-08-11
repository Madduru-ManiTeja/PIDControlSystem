% System Parameters
m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

% Transfer Function G1(s)
nump = [(m1 + m2) b2 k2];
denp = [(m1 * m2) (m1 * (b1 + b2)) + (m2 * b1) (m1 * (k1 + k2)) + (m2 * k1) + (b1 * b2) (b1 * k2) + (b2 * k1) k1 * k2];
G1 = tf(nump, denp);

% Transfer Function G2(s)
num1 = [-(m1 * b2) -(m1 * k2) 0 0];
den1 = [(m1 * m2) (m1 * (b1 + b2)) + (m2 * b1) (m1 * (k1 + k2)) + (m2 * k1) + (b1 * b2) (b1 * k2) + (b2 * k1) k1 * k2];
G2 = tf(num1, den1);

% Transfer Function F(s)
numf = num1;
denf = nump;
F = tf(numf, denf);

% PID Controller Parameters
Kd = 208025;
Kp = 832100;
Ki = 624075;
C = pid(Kp, Ki, Kd);

% Closed-Loop System
sys_cl = F * feedback(G1, C);

% Time Vector and Step Response
t = 0:0.05:5;
figure;
step(0.1 * sys_cl, t)
title('Response to a 0.1-m Step under PID Control')

% Root Locus Design
z1 = 1;
z2 = 3;
p1 = 0;
s = tf('s');
C = ((s + z1) * (s + z2)) / (s + p1);
figure;
rlocus(C * G1)
title('Root Locus with PID Controller')
[k, poles] = rlocfind(C * G1);

% Adjusting PID Gains and Re-simulating
Kd = 2 * Kd;
Kp = 2 * Kp;
Ki = 2 * Ki;
C = pid(Kp, Ki, Kd);
sys_cl = F * feedback(G1, C);
figure;
step(0.1 * sys_cl, t)
title('Response to a 0.1-m Step w/ High-Gain PID')

% Adjust Axis for Comparison
axis([0 5 -.01 .01])
