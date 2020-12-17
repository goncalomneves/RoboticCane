close all
clear
clc

%% Model 10 - Motor with rod

% U = [];
% save('controlinput.mat','U');

tspan = 0:.001:10; % Time span

% No ref
% ref(1:length(tspan)) = 0;

% Constant ref
% ref(1:length(tspan)) = pi/20;

% Variable ref
ref(1:(length(tspan)/5)) = 0;
ref((length(tspan)/5):(length(tspan)*2/5)) = pi/80;
ref((length(tspan)*2/5):(length(tspan)*3/5)) = pi/20;
ref((length(tspan)*3/5):(length(tspan)*4/5)) = pi/40;
ref((length(tspan)*4/5):length(tspan)) = -pi/20;

%% Simlate the model

y0 = [0; 0; 0; 0];
% y0 = [0; 0; pi*0.1; 0];

[t,y] = ode45(@(t,y)model(y,ref,t),tspan,y0);

% figure('Renderer', 'painters', 'Position', [10 10 480 270]);
% plot(t,y)
% title('State of the rod');
% xlabel('Time(s)');
% l = legend('$x[m]$','$\dot{x}[m/s]$','$\theta [rad]$','$\dot{\theta} [rad/s]$');
% set(l, 'Interpreter', 'latex');

figure('Renderer', 'painters', 'Position', [10 10 480 270]);
% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
plot(t,y(:,1),t,y(:,3),t,ref)
% plot(t,y(:,1),'-k',t,y(:,3),':k',t,ref,'--k')
title('State of the rod');
xlabel('Time(s)');
l = legend('$x[m]$','$\theta [rad]$','$\theta_{ref} [rad]$');
set(l, 'Interpreter', 'latex');

% figure('Renderer', 'painters', 'Position', [10 10 480 270]);
% % figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% plot(t,y(:,1),'-k',t,y(:,2),'--k',t,y(:,3),':k',t,y(:,4),'-.k')
% title('State of the cane');
% xlabel('Time(s)');
% l = legend('$x[m]$','$\dot{x}[m/s]$','$\theta [rad]$','$\dot{\theta} [rad/s]$');
% set(l, 'Interpreter', 'latex');

% load('controlinput.mat');
% U_actual = interp1(1:length(U),U,1:length(tspan));
% 
% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% plot(t,U_actual)
% title('Control Input');
% xlabel('Time(s)');
% ylabel('u(V)');
% l = legend('u');
% set(l, 'Interpreter', 'latex');