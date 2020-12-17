clear all
close all
clc

%% Variables

m = 1; % Mass of pendulum
M = 5; % Mass of cart
L = 2; % Length of pendulum
d = 1; % Friction (damping)
g = -9.8; % Gravity value

tspan = 0:.001:10; % Time span

%% To simulate the body wight on the cane, uncomment the following part

% weight = 70; % User's weight
% 
% % f = 0.5; % Walking frequency
% % F_vert = g*(weight/4)*sin(2*pi*f*tspan)+g*(1+(weight/4)); % Vertical force(Down direction -> negative)
% 
% g = -9.8 - ((weight/2)*9.8);

%% System model

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -d/(M*L) -(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; 1/(M*L)];

eig(A)

% Cost parameters
Q = [1000 0 0 0;
    0 1000 0 0;
    0 0 1000 0;
    0 0 0 1000];

R = .0001;

det(ctrb(A,B))

% Obtain gains with LQR
K = lqr(A,B,Q,R);

%% Simulate model

y0 = [0; 0; pi+pi*.1; 0];

[t,y] = ode45(@(t,y)model(y,m,M,L,g,d,-K*(y-[0; 0; pi; 0])),tspan,y0);
% [t,y] = ode45(@(t,y)model(y,m,M,L,F_vert,d,-K*(y-[1; 0; pi; 0]),t),tspan,y0);

figure('Renderer', 'painters', 'Position', [10 10 960 540]);
plot(t,y)
title('State of the cane');
xlabel('Time(s)');
l = legend('$x [m]$','$v [m/s]$','$\theta [rad]$','$\omega [rad/s]$');
set(l, 'Interpreter', 'latex');