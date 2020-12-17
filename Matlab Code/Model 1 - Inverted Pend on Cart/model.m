function dy = cartpend(y,m,M,L,g,d,u,t)

% y - State
% m - Mass of pendulum
% M - Mass of cart
% L - Length of pendulum
% g - Gravity value
% d - Friction (damping)
% u - Force on the cart in x direction


Sy = sin(y(3));
Cy = cos(y(3));
D = m*L*L*(M+m*(1-Cy^2));

dy(1,1) = y(2);
dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
dy(3,1) = y(4);
dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u +.01*randn;

% dy(1,1) = y(2);
% dy(2,1) = (-d/M)*y(2) + (-m*g/M)*y(3) + (1/M)*u;
% dy(3,1) = y(4);
% dy(4,1) = (-d/(M*L))*y(2) + (-(m+M)*g/(M*L))*y(3) + (1/(M*L))*u;