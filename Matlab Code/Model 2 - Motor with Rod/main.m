close all
clear
clc

%% Model 9 - Motor with rod

% This part saves the variables in a file
U = [];
Y = [];
T = [];
save('controlinput.mat','U','Y','T');
tspan = 0:.001:5; % Time span

% No ref
% ref(1:length(tspan)) = 0;

% Constant ref
ref(1:length(tspan)) = pi/20;

% Variable ref
% ref(1:(length(tspan)/5)) = 0;
% ref((length(tspan)/5):(length(tspan)*2/5)) = 0;
% ref((length(tspan)*2/5):(length(tspan)*3/5)) = pi/20;
% ref((length(tspan)*3/5):(length(tspan)*4/5)) = pi/40;
% ref((length(tspan)*4/5):length(tspan)) = -pi/20;

%% Simulate the model

y0 = [-pi/2; 0];

[t,y] = ode45(@(t,y)model(y,ref,t),tspan,y0);

figure('Renderer', 'painters', 'Position', [10 10 960 540]);
plot(t,y,t,ref)
title('State of the rod');
xlabel('Time(s)');
l = legend('$\theta [rad]$','$\omega [rad/s]$','$\theta_{ref} [rad]$');
set(l, 'Interpreter', 'latex');

load('controlinput.mat');
figure('Renderer', 'painters', 'Position', [10 10 960 540]);
yyaxis left
plot(T,U)
yyaxis right
plot(T,Y)
title('Control Input');
xlabel('Time(s)');
l = legend('u[V]','$x[m]$','$\theta [rad]$');
set(l, 'Interpreter', 'latex');