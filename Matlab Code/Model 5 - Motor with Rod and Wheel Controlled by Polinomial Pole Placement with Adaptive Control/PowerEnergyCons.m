close all
clear
clc
% This model returns an approx value of energy consumption of the system


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
% rlocus(sysctr);

% figure();
% step(sysctr);

num_xctrl_tf = P;
den_xctrl_tf = L;

%% Creating Force Signals

User_Weight = 70; % User Weight
g = -9.80621; % Gravity value
% Walking_Freq = 2; % Walking Frequency
Fmed = 0.25*User_Weight*g; % Mean Force Applied -> Max = 30%  Min = 20% Med = 25%
Fvar = 0.1*User_Weight*g; % Force variation -> Max - Min = 10%
tspan = 0:.001:30; % Time span

Walking_Freq_vec = 0:0.04:4; % Walking Frequency
X_Force_vec = 0:0.08:8;

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

alpha = 0.29;

%% Run Simulation

Power = [];
Energy = [];

for Walking_Freq = 0:0.04:4 % Walking Frequency
    
X_Force = 3/sqrt(2)*sqrt(Walking_Freq); % Force on X axis

Simulation_Time = 30;
sim('wheel_and_bar9',Simulation_Time);

L_u = length(u);
Power = [Power (norm(u)^2)/L_u]; %Power Consumed
Energy = [Energy ((norm(u)^2)/L_u)*3600/30]; %Energy Consumed

end

figure('Renderer', 'painters', 'Position', [10 10 960 540]);
yyaxis left
plot(Walking_Freq_vec,Power);
% ylim([0 3]);
ylabel('Power [W]');
yyaxis right
plot(Walking_Freq_vec,Energy);
% ylim([0 20]);
ylabel('Energy [Wh]');
title('Power & Energy Consumption depending on Walking Frequency');
xl = xlabel('Walking Frequency [Hz]');