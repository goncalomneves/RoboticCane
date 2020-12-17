function y_ref = reference(forces)

fx = forces(1);

fz = forces(2);

lc = 0.4; % Length from the axle to the center of mass of the rod [m]
l = 0.8; % Length of the rod [m]
r = 0.1; % Radius of the wheek [m]
m = 0.5; % Mass of cane [kg]
g = 9.80621; % Gravity constant [m^2/s]

psi = atan(l*fx/((lc*m*g)-l*fz));

theta_ref = asin(((tan(psi))/(1+tan(psi).^2))*((r/l)+sqrt((1-((r^2)/(l^2)))*tan(psi)+1)));

pos_ref = (l)*sin(theta_ref);

y_ref = [pos_ref;theta_ref];