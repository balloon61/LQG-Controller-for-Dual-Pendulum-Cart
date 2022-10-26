%% System Parameters %%
syms M m1 m2 l1 l2

%% Parameters  Values%%

g = 10; % gravity
% M = 1000; % Cart Mass
% m1 = 100; % Pendulum 1 Mass
% m2 = 100; % Pendulum 2 Mass
% % l1 = 20; % Pendulum 1 Length
% % l2 = 10; % Pendulum 2 Length

%% State spaee model %%

A = [0 1 0                0 0                  0;
     0 0 -m1*g/M          0 -m2*g/M            0;
     0 0 0                1 0                  0;
     0 0 (-m1*g-M*g)/(M*l1) 0 -m2*g/(M*l1)       0;
     0 0 0                0 0                  1;
     0 0 -m1*g/(M*l2)     0 (-m2*g-M*g)/(M*l2) 0];
B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];
C = [];
D = [];

%% Check Controllability %%

% Controllability(A,B)
Controllability_check(A,B);

%% Controllability %%
function [C] = Controllability_check(A, B)
    C = [B A*B A*A*B A*A*A*B A*A*A*A*B A*A*A*A*A*B]
    rank(C)
    det(C)
    
end
