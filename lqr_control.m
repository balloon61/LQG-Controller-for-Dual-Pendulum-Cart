%% System Parameters %%
syms M m1 m2 l1 l2 g
% Parameters
g = 10; % gravity
M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
%% Nonlinear System Model %%

% NL_A = [xc_dot]
% NL_B = []
% NL_C = []
% NL_D = []
% x1_dot = x2;
% x2_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) / (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M);
% x3_dot = x4;
% x4_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) /(l1 * (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M)) - g*sin(x3)/l1;
% x5_dot = x6;
% x6_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) /(l2 * (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M)) - g*sin(x5)/l2;

%% State Spaee Model %%

A = [0 1 0                0 0                  0;
     0 0 -m1*g/M          0 -m2*g/M            0;
     0 0 0                1 0                  0;
     0 0 (-m1*g-M*g)/(M*l1) 0 -m2*g/(M*l1)     0;
     0 0 0                0 0                  1;
     0 0 -m1*g/(M*l2)     0 (-m2*g-M*g)/(M*l2) 0];
B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];
C = [];
D = [];

%% LQR controller %%
R = 10;
Q = [100000 0 0 0 0 0;
     0 100000 0 0 0 0
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'xc' 'xc_dot' 'theta1' 'theta1_dot' 'theta2' 'theta2_dot'};
% inputs = {'r'};
% outputs = {'x'; 'phi'};

sys = ss(Ac,B,C,0,'statename',states);

t = 0:0.001:1000; % Time
r = zeros(size(t)); % Reference
x0 = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1]; % Initial Condition
[y,t,x] = lsim(sys,r,t,x0);

%% Linear Figures %%

subplot(2,3,1);
plot(t,x(:,1))
title('State 1: x')
subplot(2,3,4);
plot(t,x(:,2))
title('State 2: x dot')
subplot(2,3,2);
plot(t,x(:,3))
title('State 3: theta1')
subplot(2,3,5);
plot(t,x(:,4))
title('State 4: theta1 dot')
subplot(2,3,3);
plot(t,x(:,5))
title('State 5: theta2')
subplot(2,3,6);
plot(t,x(:,6))
title('State 6: theta2 dot')
hold on;
%% Nonlinear Figures %%

% subplot(2,3,1);
% plot(t,x(:,1))
% title('State 1: x')
% subplot(2,3,4);
% plot(t,x(:,2))
% title('State 2: x dot')
% subplot(2,3,2);
% plot(t,x(:,3))
% title('State 3: theta1')
% subplot(2,3,5);
% plot(t,x(:,4))
% title('State 4: theta1 dot')
% subplot(2,3,3);
% plot(t,x(:,5))
% title('State 5: theta2')
% subplot(2,3,6);
% plot(t,x(:,6))
% title('State 6: theta2 dot')

%% Controllability %%
function [C] = Controllability_check(A, B)
    C = [B A*B A*A*B A*A*A*B A*A*A*A*B A*A*A*A*A*B];
end
function ode()
% x1_dot = x2;
% x2_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) / (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M);
% x3_dot = x4;
% x4_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) /(l1 * (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M)) - g*sin(x3)/l1;
% x5_dot = x6;
% x6_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) /(l2 * (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M)) - g*sin(x5)/l2;
end
