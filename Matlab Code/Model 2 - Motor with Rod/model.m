function [dy, u] = model(y,ref,t)

% Model 9 - Motor with rod

% y - State
% u - Force on the cart in x direction

M_r = 0.5; % Mass of rod
L = 0.8; % Length of pendulum
d = 0.025; % Friction (damping)
g = -9.80621; % Gravity value

ref_actual = interp1(1:length(ref),ref,(t*1e+3)+1);
% time = interp1(1:100001,1:100001,(t*1e+3)+1);

% u = -K*(y-[ref_actual; 0]);
% 
% %Sy = sin(y(1));
% 
% dy(1,1) = y(2);
% %dy(2,1) = (u-((M_r*g*L*Sy-d*y(2))/2))/(((M_r*(L^2))/3)+(M_w*(R_w^2))/2);
% dy(2,1) = (u-((M_r*g*L*y(1))/2)-d*y(2))/(((M_r*(L^2))/3)+(M_w*(R_w^2))/2);

%% Controller

D1 = (M_r*(L^2))/3;

A = [0 1 ;
    -(M_r*g*L)/(2*D1) -d/D1];

B = [0; 1/D1];

C = [1 0];

D = [0];

% R = 0  -> Pouca importância à energia (controlador nervoso)
% R = infinito -> Muita importância à energia (controlador lento)

% Botões de ajuste do LQR: matrizes Q e R
Q = eye(2); % Fixo, não se altera
R = 1e-4; % Botão de ajuste do custo da energia

% Gerar o ganho do controlador
K = lqr(A, B, Q, R);

%% Calculate control signal and new states

u = -K*(y-[ref_actual; 0]);

dy = A*y+B*u;

%% Save variables
load('controlinput.mat')
U = [U; u];
Y = [Y; y(1)];
T = [T; t];
save('controlinput.mat','U','Y','T');

fprintf('%f \n',t);

%% Plots

% %pzmap
% figure('Renderer', 'painters', 'Position', [10 10 1400 540]);
% % figure;
% subplot(1,2,1);
% pzmap(ss(A,B,C,D));
% title('Open-Loop Pole-Zero Map');
% % figure;
% subplot(1,2,2);
% pzmap(feedback((tf(ss(A,B,C,D))*tf(K)),tf(C'),-1));
% title('Closed-Loop Pole-Zero Map');