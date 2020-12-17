close all
clear
clc

%% Control of Angle

% Theta TF:
num_theta = [9.3750   -3.2225];
den_theta = [1.0000  0.2344  -18.5155   0];

% Theta Controller TF1:
num_thetactrl_tf1 = [1 20 101];
den_thetactrl_tf1 = [1 200 10000];

% Theta Controller TF2:
num_thetactrl_tf2 = 60*[9.375 -3.2225];
den_thetactrl_tf2 = [1 -3];

% Ang_sys = tf(num_theta,den_theta);
% C1 = tf(num_thetactrl_tf1,den_thetactrl_tf1);
% C2 = tf(num_thetactrl_tf2,den_thetactrl_tf2);
% sys_ang_ctr1 = feedback(Ang_sys,1);
% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% pzmap(sys_ang_ctr1);
% sys_ang_ctr2 = feedback(Ang_sys*C1,1);
% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% pzmap(sys_ang_ctr2);
% sys_ang_ctr3 = feedback(Ang_sys*C1*C2,1);
% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% pzmap(sys_ang_ctr3);

%% Control of Position

% X TF:
num_x = [6.25 -0.8789 -114.9];
den_x = [1 0.2344 -18.52 0 0];

% X Controller TF (with polynomial pole placement):
Apr = den_x;
Bpr = num_x;

% M=[Apr 0;0 Apr;Bpr 0 0 0;0 Bpr 0 0;0 0 Bpr 0;0 0 0 Bpr]';
M=[Apr 0 0 0;0 Apr 0 0; 0 0 Apr 0; 0 0 0 Apr;0 0 Bpr 0 0 0;0 0 0 Bpr 0 0;0 0 0 0 Bpr 0;0 0 0 0 0 Bpr]';

% x = conv(conv([1 199.9886 1.0001e+04],[1 160 6401]),conv([1 4 4.5],[1 4.4223]))'; %FUNCIONA! - C = [-18285577 -83186110 -12776734 -11087457],[1 368 114336241 480344831]
x = conv(conv([1 12 37],[1 16 65]),conv([1 4 4.25],[1 2.4223]))'; %8.503 e -3.981

theta=M\x;

% Lprime=theta(1:3)';
% Pprime=theta(4:7)';
% L=conv(Lprime,[1 0 1]);
% P=[0 Pprime];

L=theta(1:4)';
P=theta(5:8)';
C=tf(P,L);

% roots(conv(A,L)+conv(B,P))

sys = tf([6.25 -0.8789 -114.9],[1 0.2344 -18.52 0 0]); %TF de x

% sysctr = feedback(C*sys,1);
% pole(sysctr)
% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% % rlocus(sysctr);
% pzmap(sysctr);

% figure();
% step(sysctr);

num_xctrl_tf = P;
den_xctrl_tf = L;

%% Convert Controllers to descrete time for Arduíno
Ctrl_angle = c2d((tf(num_thetactrl_tf1,den_thetactrl_tf1)*tf(num_thetactrl_tf2,den_thetactrl_tf2)),0.1);
Ctrl_position = c2d((tf(num_xctrl_tf,den_xctrl_tf)),0.1);

%% Creating Force Signals

User_Weight = 70; % User Weight
g = -9.80621; % Gravity value
% Walking_Freq = 2; % Walking Frequency
Fmed = 0.25*User_Weight*g; % Mean Force Applied -> Max = 30%  Min = 20% Med = 25%
Fvar = 0.1*User_Weight*g; % Force variation -> Max - Min = 10%
tspan = 0:.001:30; % Time span

% Z_Force = [tspan;Fvar*sin(2*pi*Walking_Freq*tspan)+Fmed]'; % Vertical Force

% X_Force = 3/sqrt(2)*sqrt(Walking_Freq); % Force on X axis

% % Slow Walking
% Walking_Freq = 0.5; % Walking Frequency
% X_Force = 1.5;

% % Average Walking
% Walking_Freq = 2; % Walking Frequency
% X_Force = 3;

% Quick Walking
Walking_Freq = 4; % Walking Frequency
X_Force = 4.2426;

% plot(tspan,Z_Force,tspan,X_Force);

%% Adding Noise

% NoiseSignal_X = 0.2*X_Force*rand(1, length(tspan)); % At max 20% of the X force
% NoiseSignal_Z = 0.2*Fvar*rand(1, length(tspan)); % At max 20% of the Z force

%% Defining Model

M_r = 0.5; % Mass of rod
M_w = 1.1; % Mass of wheel
R_w = 0.1; %Radious of wheel
L = 0.8; % Length of pendulum
d = 0.025; % Friction (damping)
g = -9.80621; % Gravity value

D1 = M_w+M_r;
D2 = (M_r*(L^2))/3;

A = [0 1 0 0 ;
    0 0 0 (-M_r*L)/D1;
    0 0 0 1;
    0 (-M_w*R_w)/(2*D2) (-M_r*g*L)/(2*D2) -d/D2];


B = [0;
    (1/R_w)/D1;
    0;
    1/D2];

C = [1 0 0 0;
    0 0 1 0];

D = [0;
    0];

%% Controller Weights

% alpha = 0  -> Pouca importância ao controlo da posição e muita importância ao controlo do ângulo
% alpha = 1 -> Muita importância ao controlo da posição e pouca importância ao controlo do ângulo

% alpha = 0.21;

% betta = 0.3; %35-65%
% betta = 0.7; %15-85%
% betta = 0.4; %30-70%
betta = 1; %0-100%

%% Run Simulation

% rmse = [];
% rmse_pos = [];
% rmse_ang = [];
% max_pos = [];
% max_ang = [];
% for betta = 0:0.1:1

Simulation_Time = 30;
sim('wheel_and_bar11',Simulation_Time);

% L_u = length(u);
% Power = (norm(u)^2)/L_u; %Power Consumed
% Energy = Power*3600/30;

% (y - yhat);    % Errors
% (y - yhat).^2;   % Squared Error
% mean((y - yhat).^2);   % Mean Squared Error
% rmse = [rmse sqrt(mean(mean((y - ref).^2)))];  % Root Mean Squared Error
% rmse_pos = [rmse_pos sqrt(mean(y(:,1) - ref(:,1)).^2)];  % Root Mean Squared Error of Position
% rmse_ang = [rmse_ang sqrt(mean(y(:,2) - ref(:,2)).^2)];  % Root Mean Squared Error of Angle
% max_pos = [max_pos max(y(:,1))];
% max_ang = [max_ang max(y(:,2))];

RMSE = sqrt(mean(mean((y - ref).^2))); % Root Mean Squared Error
RMSE_pos = sqrt(mean(y(:,1) - ref(:,1)).^2);  % Root Mean Squared Error of Position
RMSE_ang = sqrt(mean(y(:,2) - ref(:,2)).^2);  % Root Mean Squared Error of Angle

figure('Renderer', 'painters', 'Position', [10 10 300 200]);
yyaxis left
plot(t,y(:,1),t,ref(:,1));
ylabel('Position[m]');
yyaxis right
plot(t,y(:,2),t,ref(:,2));
ylabel('Angle[rad]');
% title('State of the rod');
title('Limits: 35 - 65%');
xlabel('Time[s]');
l = legend('$x[m]$','$x_{ref} [m]$','$\theta [rad]$','$\theta_{ref} [rad]$');
set(l, 'Interpreter', 'latex');

% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% plot(t,f);
% title('Force Applied');
% xlabel('Time(s)');
% l = legend('$F_x[N]$','$F_y[N]$');
% set(l, 'Interpreter', 'latex');

figure('Renderer', 'painters', 'Position', [10 10 300 200]);
plot(t,alpha);
title('Alpha');
xlabel('Time(s)');

% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% plot(t,y,t,u);
% title('State of the rod with input');
% xlabel('Time(s)');
% l = legend('$x[m]$','$\theta [rad]$','$u$');
% set(l, 'Interpreter', 'latex');

% end
% 
% alpha = 0:0.01:1;
% RMSE = [rmse ; rmse_pos ; rmse_ang ; max_pos ; max_ang ; alpha]';
% min(RMSE(:,1))
% min(RMSE(:,2))
% min(RMSE(:,3))
% min(RMSE(:,4))
% min(RMSE(:,5))
% 
% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% yyaxis left
% plot(alpha,rmse,alpha,rmse_pos,alpha,rmse_ang);
% ylim([0 3]);
% ylabel('Root Mean Squared Error');
% yyaxis right
% plot(alpha,max_pos,alpha,max_ang);
% ylim([0 20]);
% ylabel('Maximum value of each state');
% title('Controller performance');
% xl = xlabel('$\alpha$');
% l = legend('$Mean RMSE$','$RMSE of Position$','$RMSE of Angle$','$Max Position [m]$','$Max Angle [rad]$');
% set(xl, 'Interpreter', 'latex');
% set(l, 'Interpreter', 'latex');

% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% pzmap(tf(num_theta,den_theta))
% title('Pole-Zero Map of the angle component');
% 
% figure('Renderer', 'painters', 'Position', [10 10 960 540]);
% pzmap(tf(num_x,den_x))
% title('Pole-Zero Map of the position component');