function dy = model(y,ref,t)

% Model 10 - Motor with rod

% y - State
% u - Force on the cart in x direction

M_r = 0.5; % Mass of rod
M_w = 1.1; % Mass of wheel
R_w = 0.1; %Radious of wheel
L = 0.8; % Length of pendulum
d = 0.025; % Friction (damping)
g = -9.80621; % Gravity value

ref_actual = interp1(1:length(ref),ref,(t*1e+3)+1);

%% Create the model

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


%% Controller

% R = 0  -> Pouca importância à energia (controlador nervoso)
% R = infinito -> Muita importância à energia (controlador lento)

% Botões de ajuste do LQR: matrizes Q e R
% Q = eye(2); % Fixo, não se altera
Q = C'*C; % Fixo, não se altera
R = 1e-4; % Botão de ajuste do custo da energia

% Gerar o ganho do controlador
K = lqr(A, B, Q, R);

%% Calculate the control signal and new states

% u = -K*(y-[y(1); 0; ref_actual; 0]); % apenas controlo do ângulo

u = -K*(y-[0; 0; ref_actual; 0]);

dy = A*y+B*u;

%% Save control input
% load('controlinput.mat')
% U = [U; u];
% save('controlinput.mat','U');

% fprintf('%f \n',t);

%% Plots
% 
% figure('Renderer', 'painters', 'Position', [10 10 1400 540]);
% 
% % figure;
% subplot(1,2,1);
% pzmap(ss(A,B,C,D));
% title('Open-Loop Pole-Zero Map');
% 
% % figure;
% subplot(1,2,2);
% pzmap(ss(A-B*K,B,C,D));
% title('Closed-Loop Pole-Zero Map');