% Ramon Garcia-Figueroa, 11/12/2021, Linear Control System, Lab simulation 4
% Part 1—Design and Simulation: Write a Matlab/Octave script to use the Ziegler Nichols-Tuning 
% Method to optimize the PID Controller for a Process and then Implement the Control System 
% using Linear Integrated Circuits. 

clc;
close all;
clear all;
tic

% define s
 
s = tf('s');
 
% a) open-loop transfer function

G = 1/(s*(s+6)*(s+2))
 
% Plot the root locus to identify the critical/ultimate gain
figure(1);
rlocus(G) 

% b) The Routh-Hurtwitz: K = 96

% display root locus plot of the specified SISO system
%[rldata,K] = rlocus(G,0.1,5)

% obtaining root locations r associated with various values of the gain K
[r,K] = rlocus(G)
 
% define ultimate gain and ultimate time period
Ku = 30.2;     % ultimate gain, and damping at zero

Ts = 4;        % settling time under 5sec
 
Pu = 2*pi/Ts;  % ultimate period: oscillations
 
% define controller gains from Table of Ziegler-Nicholas PID 
fprintf('Controller gains from Table of Ziegler-Nicholas PI:\n');  
Kp = 0.6*Ku;
 
Ki = 1.2*Ku/Pu;

Kd = 0.6*Ku*Pu/8;
 
% by using Ziegler-Nicholas formulas value obatin are:
 
Kp =   18.120
 
Ki =   12.920
 
Kd =   6.350
fprintf('By using Ziegler-Nicholas formulas value obtained are:\n');  
% define the controller: Proportional-plus-integral-derivative(PID) table e)
 
Gc = Kp+Ki/s+Kd*s;
 
% plot the step response
figure(2);
G1 = series(Gc,G);
G1_l = feedback(G1,1);
step(G1_l);grid on;
%step(feedback(Gc*G,1));grid on;
title('Step Response with PID controller');

% plot step response for change in disturbance
figure(3);
G2 = feedback(G,Gc);
step(G2);grid on;
title('Respond to unit step change in Disturbance with PID controller')

% plot ramp response
figure(4);
t = [0:0.1:10];
u = t; % input ramp
G1 = series(Gc,G);
G1_l = feedback(G1,1);
[Y,t1] = lsim(G,u,t);     % ramp response of transfer function 
[Y2,t2] = lsim(G1_l,u,t); % ramp response of transfer function with PID controller
subplot(221),plot(t1,Y),grid;title('Ramp Input tf')
subplot(222),plot(t2,Y2),grid;title('Ramp Input PID')
%Yr = lsim(G1_l,u,t);
%title('Ramp Input')
%ramp(G1_l,t)
%ramp(G1,t)
figure(5);
t = 0:0.1:2;
u = t; % input ramp
[y,t,x] = lsim(G1_l,u,t);
plot(t,y,'y',t,u,'m')
xlabel('Time sec')
ylabel('Amplitude')
title('Input-purple, Output-yellow')


%New controller gains are:
Kp = 20
 
Ki = 6.4
 
Kd =14.

toc


