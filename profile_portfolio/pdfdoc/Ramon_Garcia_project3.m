 % Ramon Garcia-Figueroa, 10/28/2021, Linear Control System, lab simulation 3
% Write a script to use an Integral square error optimization method to design an optimal gain to control a process until it attains a set of desirable goals

clc;
clear;

%%num = [1 6],[1 4];    % Controller
%%den = [1],[1 4 8 0];  % Process

% open loop transfer function
G = tf([1 6],[1 4]);
C = tf([1],[1 4 8 0]);
sys = G*C;

% define s
s = tf('s');

% loop transfer function
%%sys1 = tf([num],[den]);

% finding closed loop transfer function
%%sys2 = feedback(sys1,1,-1);
sys2 = feedback(sys,[1])

% margin 
Gm = margin(sys)
Gm1 = margin(sys2)

% Routh Hurwitz
syms K s
g = K*(s+6);
gc = s^4+8*s^3+24*s^2+s*(K+32)+K*6
ts = g/gc

% Create Routh array (RH)
RH = [1 24 6*K;
      8 K+32 0];
b1 = (RH(2,1)*RH(1,2)-RH(1,1)*RH(2,2))/(RH(2,1));
%b2 = (RH(1,3));
b2 = (RH(2,1)*RH(1,3)-RH(1,1)*RH(2,3))/(RH(2,1));
b3 = 0;
c1 = (b1*RH(2,2)-RH(2,1)*b2)/b1;
c2 = (b1*RH(2,3)-RH(2,1)*b3)/b2;
c3 = 0;
d1 = (c1*b2-b1*c2)/c1;
d2 = (c1*b3-b1*c3)/c1;
d3 = 0;
% Full table of RH 
RHf = [1 24 6*K;
       8 K+32 0;
simplify(b1) b2 b3;
simplify(c1) c2 c3;
simplify(d1) d2 d3]


% Full table
RHf_1 = [1 17+5*K 6*K;
       8 10+17*K 0;
simplify(b1) b2 b3;
simplify(c1) c2 c3;
simplify(d1) d2 d3]

% vapsolve=solve eqautions numerically: K value for stability to staisfy condition when b1=0, c1=0, d1=0
K_1 = vpasolve(b1,K)
K_2 = vpasolve(c1,K)
K_3 = vpasolve(d1,K)

% root locus of dynamic system
figure(1);
rlocus(sys);

% display root locus plot of the specified SISO system
[rldata,K] = rlocus(sys2,0.1,0,10)
[rldata_1,K_1] = rlocus(sys,0.1,0,1)

% obtaining root locations r associated with various values of the gain K
[r,K] = rlocus(sys2);
[r1,K1] = rlocus(sys);

% pole function to compute the closed-loop control system poles of the system
p = pole(sys2)
p1 = pole(sys)

% plot unit step response
figure(2);
t = [0:0.1:10];
step(sys2,t)
step(sys2)

figure(3);
t = [0:0.1:10];
step(sys,t)
step(sys)

% Integral time square error perfomance index
G = zpk([-6],[-2+2i, 0, -4, -2-2i],1)
ise = 1;                         % integral square error
iste = 1;                        % intergral time square error
t = [0:0.1:10];                  % time control system for 10 seconds of step input
for a = 1:1:100
  K = a/10;
  H = feedback(K*G,1);           % feedback system
  [Y,T] = step(H,t);             % step response
  figure(4);
  hold on
  plot(T,Y);
  title('Integral time square error perfomance index');xlabel('Time first 10sec');ylabel('y(t)');
  grid on;
  ise(a) = trapz((1-Y).^2);       % trapezoidal numerical integration, NOTE: subtracting 1 from stored response yields a bew array called error. Then square error and use trapz to integrate.
  iste(a) = trapz(T,(1-Y).^2);    % compute the integral
 end
 
figure(5);
plot(ise);grid;title('Intergral Square Error');xlabel('Time (s)');ylabel('y(t)');
[Ys1,Ts1] = min(iste)             % returns the mininimun elements of an array
opt = feedback(Ts1/10*G,1)        % (optimize)=system parameters adjusted so index reaches an extremum, commonly a mininimun value. Feedback system.

figure(6);
[Ys2,Ts2] = step(opt,t);          % step response 
figure(6);
plot(Ts2,Ys2);grid on;title('Optimize Step Response using Intergral Square Error');xlabel('Normalize Time');ylabel('Normalize Response');

c = ones(size(t));
u = t.^2;
v = t;
[Y,T] = lsim(opt,c,t);            % simulated response data
[X,T] = impulse(opt,t);           % calculate unit-impulse response of dynamic system model
[W,T] = lsim(opt,v,t);
[Z,T] = lsim(opt,u,t);

figure(7);
subplot(221),plot(t,Y),title('Step Response');
subplot(222),plot(t,X),title('Impluse');
subplot(223),plot(t,Z),title('Parabolic Response');
subplot(224),plot(t,W),title('Ramp Response');
grid;
