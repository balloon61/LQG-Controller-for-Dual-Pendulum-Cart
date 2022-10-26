%% System Parameters %%
syms M m1 m2 l1 l2 g
% Parameters
g = 10; % gravity
M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;

%% State Spaee Model %%
% equailbrium points = (0 0 0 0 0 0)
A = [0 1 0                0 0                  0;
     0 0 -m1*g/M          0 -m2*g/M            0;
     0 0 0                1 0                  0;
     0 0 (-m1*g-M*g)/(M*l1) 0 -m2*g/(M*l1)     0;
     0 0 0                0 0                  1;
     0 0 -m1*g/(M*l2)     0 (-m2*g-M*g)/(M*l2) 0];
B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];
C = [];
D = [];

%% Eigenvalues

eo = eig(A)

R = 10;
Q = [10000 0 0 0 0 0;
     0 100000 0 0 0 0
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];

ec = eig(Ac)