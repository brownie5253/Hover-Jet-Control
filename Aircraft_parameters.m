%% define parameters
g = 9.8;
c = 0.05; %damping
m = 4;
j = 0.0475;
r = 0.25;

Params = [g c m j r];

%% define state space Matrices Equilib A:

% Linearised equilibrium A
A = [0 0 0 1 0 0; 
     0 0 0 0 1 0; 
     0 0 0 0 0 1; 
     0 0 -g -c/m 0 0; 
     0 0 0 0 -c/m 0; 
     0 0 0 0 0 0];
 
B = [0 0; 
     0 0; 
     0 0; 
     1/m 0; 
     0 1/m; 
     r/j 0];
 
C = eye(3,6);
D = zeros(3,2);

u_bar = [0 0];
x_bar = [0 0 0 0 0 0]'; % equilibrium point (x y theta x' y' theta') change these to set final position
x0 = [0.5 1.5 0 0 0 0]'; %inital condition (x y theta x' y' theta') change there to set starting pos of aircraft
x_inital = x0-x_bar;