

clc;
close all;
clear all;
tic

% define s
 
s = tf('s');
 
% a) open-loop transfer function

G = 1/(s*(s+6)*(s+2));
 
% Plot the root locus to identify the critical/ultimate gain
figure(1);
rlocus(G) 
 
% define ultimate gain and ultimate time period
Ku = 30.2;     % ultimate gain, and damping at zero

Ts = 4;        % settling time under 5sec
 
Pu = 2*pi/Ts;  % ultimate period: oscillations
 
% define controller gains form Table of Ziegler-Nicholas PID 
 
Kp = 0.6*Ku;
 
Ki = 1.2*Ku/Pu;

Kd = 0.6*Ku*Pu/8;
 
% by using Ziegler-Nicholas formulas value obatin are:
 
Kp =   18.120
 
Ki =   12.920
 
Kd =   6.350
 
% define the controller: Proportional-plus-integral-derivative(PID) table e)
 
Gc = Kp+Ki/s+Kd*s;
 
% plot the step response
figure(2);
G1 = series(Gc,G);
G1_l = feedback(G1,1)
step(G1_l);grid on;
%step(feedback(Gc*G,1));grid on;
title('Step Response with PID controller');

% plot step response for change in disturbance
figure(3);
G2 = feedback(G,Gc)
step(G2);grid on;
title('Respond to unit step change in Disturbance with PID controller')

% plot ramp response
figure(4);
t = 0:0.1:2;
u = t; % input ramp
[Y,t1] = lsim(G,u,t);     % ramp response of transfer function 
[Y2,t2] = lsim(G1_l,u,t); % ramp response of transfer function with PID controller
subplot(221),plot(t1,Y),grid;title('Ramp Input tf')
subplot(222),plot(t2,Y2),grid;title('Ramp Input PID')
%ramp(G1_l,t)
%ramp(G1,t)



%New controller gains are:
 
Kp = 20
 
Ki = 6.4
 
Kd =14.

toc


