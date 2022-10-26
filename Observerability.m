%% system Parameters %%
syms M m1 m2 l1 l2
% Parameters
% g = 10; % gravity
% M = 1000;
% m1 = 100;
% m2 = 100;
% l1 = 10;
% l2 = 20;
%% State spaee model %%

A = [0 1 0                0 0                  0;
     0 0 -m1*g/M          0 -m2*g/M            0;
     0 0 0                1 0                  0;
     0 0 (-m1*g-M*g)/(M*l1) 0 -m2*g/(M*l1)       0;
     0 0 0                0 0                  1;
     0 0 -m1*g/(M*l2)     0 (-m2*g-M*g)/(M*l2) 0];
B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];
C1 = [1 0 0 0 0 0];
C2 = [0 0 1 0 0 0;0 0 0 0 1 0];
C3 = [1 0 0 0 0 0;0 0 0 0 1 0];
C4 = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];

Observerability_check(A,C1); % Observerable
Observerability_check(A,C2); % Unobserverable
Observerability_check(A,C3); % Observerable
Observerability_check(A,C4); % Observerable

D = [];

%% Observerability %%
function [O] = Observerability_check(A, C)
    O = [C; C*A; C*A*A; C*A*A*A; C*A*A*A*A; C*A*A*A*A*A]
    rank(O)
    det(rank(O))
end


