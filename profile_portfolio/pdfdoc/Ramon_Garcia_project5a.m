% Ramon Garcia-Figueroa, 11/29/2021, Linear Control System, project 5

% Design a lead-lag compensator for a Control System using Frequency Domain methods 

% An automobile ignition with a unity feedback loop transfer function is shown in the block diagram 
% below. The system is to be compensated such that its response to a unit step input will have a 
% percentage overshoot of less than 15% with a settling time of less than 2s. It is required that the error 
% for a ramp input be 1% of the magnitude, while exhibiting a phase margin of 45 degrees. Perform the 
% design using the steps shown below. 

clc;
clear all;
%num = [0];
%den = [0 -1 1];
%sys = tf(num,den);
sys = zpk([],[0 -1],1)       % transfer function
figure(1);
rlocus(sys)                  % locus plot
[GM,PM,fg,fp] = margin(sys)  % compute phase margin % crossover frequency: GM=gain margin(dB), PM=phasemargin(deg),fg=freq. for phase(-180),fp=freq. for gain(0dB)
figure(2);
margin(sys)                  % phase margin
figure(3);
nyquist(sys)                 % stability design in frequency domain

t = 2;                       % settling time: given
po = 15;                     % percent shoot: given
pm = 45;                     % phase margin 45 degrees: given

zeta = sqrt((log(100/po))/(pi^2+(log(100/po))^2));
%zeta = sqrt(0.5*(1-sqrt(1-1/mp^2)))          % resonant frequency equation 8.36
Mpw = (2*zeta*sqrt(1-zeta^2))^-1;             % max frequency response
Mpt =  1 + exp(-1*zeta*pi/(sqrt(1-zeta^2)));  % resonant frequency
wn = t/zeta;                                  % wn=natural frequency 
wr = wn*(sqrt(1-2*zeta^2));                   % wr=resonant frequency
phase = pm-PM+10;                             % magnitude and phase degrees
alpha = (1-sind(phase))/(1-sind(phase));      % sinusoidal wave in degrees,compute alpha(?)
wm = 3*alpha;                                 % 3dB
Tau = wm^-1*sqrt(1/alpha);                    % polar plot equation 8.12 and time constant, logarithmic gain, compute TAO(?)

sys_lead = tf([1 Tau],[1 Tau*alpha]);         % system LEAD 
sys_feedback = feedback(sys*sys_lead,1);      % closed loop transfer function
t1 = (0:0.01:10);                             % time in second run
impulse_response = t1==0;                     % impulse response of system at zero state response
unit_step = t1>0;                             % unit step time greater than zero
ramp_response = t1.*unit_step;                % ramp respose calculation
[Y3,t3] = lsim(sys_feedback,ramp_response,t1);% impulse plotting
figure(4);grid;
plot(t3,Y3,t1,ramp_response);
title('Error of Ramp Response');
legend('t3','Y3');



