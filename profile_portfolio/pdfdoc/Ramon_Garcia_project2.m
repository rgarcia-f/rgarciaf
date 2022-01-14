% Ramon Garcia-Figueroa, 10/04/2021, Linear Control System, Lab Simulation 2
% Objective: Write a script to plot Root-Locus corresponding to the characteristic equation of a given transfer function

%example 1+(K)[(s+1)/s(s+2)(s+3)]=0 s^2+5s+6 BOOK! 
%p = [1 1];
%q = [1 5 6 0];
%[r,k]=rlocus(p,q);
%plot(r,'x')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%figure(3);
%sys1 =tf([1 1],[1 5 6 0]);
%rlocus(sys1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%figure(1);
%s = tf('s');
%g = (s^2+2*s+2)/(s*(s^4+9*s^3+33*s^2+51*s+26));
%rlocus(g);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%figure(2);
%sys = tf([2 5 1],[1 2 3]);
%rlocus(sys)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;

num = input('Enter the coeffiecients of the numerator of J(s):');    % user input
den = input('Enter the coeffiecients of the denominator of J(s):');  % user input

%%K = 1;
%%K = 0:0.1:1000;

% define s
s = tf('s');

% transfer function
sys2 = tf([num],[den]);

%%H = 1;

open_loop_zeros = roots(num);
open_loop_poles = roots(den);
plot(real(open_loop_zeros),imag(open_loop_zeros),'ro');
hold on;
plot(real(open_loop_poles),imag(open_loop_poles),'rx');

for K = 0:0.1:10;
  closed_loop_poles = pole(feedback(sys2*K,1));
  plot(real(closed_loop_poles),imag(closed_loop_poles),'bx');
end
 
hold off;
grid on;

% root locus of dynamic system
rlocus(sys2)

%%T = feedback(sys2*K,H);
%%figure(2);
%%step(T)

[rldata,K] = rlocus(sys2,0.1,0,1000)

% obtaining root locations r associated with various values of the gain K
[r,K] = rlocus(sys2);
 



 
 
 
