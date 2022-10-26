clear all
%% System Parameters %%
syms M m1 m2 l1 l2
% Parameters
g = 10; % gravity
M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
%% Nonlinear System Model %%

% Nx1_dot = x2;
% Nx2_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) / (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M);
% Nx3_dot = x4;
% Nx4_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) /(l1 * (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M)) - g*sin(x3)/l1;
% Nx5_dot = x6;
% Nx6_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) /(l2 * (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M)) - g*sin(x5)/l2;

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
     0 0 10 0 0 0;
     0 0 0 10 0 0;
     0 0 0 0 10 0;
     0 0 0 0 0 10];

%% Observer design%%


C1 = [1 0 0 0 0 0];
C2 = [0 0 1 0 0 0;0 0 0 0 1 0];
C3 = [1 0 0 0 0 0;0 0 0 0 1 0];
C4 = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];

L_T1 = acker(A',C1',[-1 -1 -1 -1 -1 -1]);
L_T3 = place(A',C3',[-1 -2 -3 -4 -5 -6]);
L_T4 = place(A',C4',[-0.61 -0.62 -0.63 -0.64 -0.65 -0.66]);
L1 = L_T1';
L3 = L_T3';
L4 = L_T4';
num = 1;
%% Observer %%

x = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1];
x_hat = [0; 0; 0; 0; 0; 0]; % Initial Condition
dt = 0.001;
Times = 1000/dt; % 
record_state_x = zeros(6,Times);
record_state_x_hat = zeros(6,Times);
record_times = zeros(1,Times);

K = lqr(A,B,Q,R);

for t = 0.0 : dt : Times*dt
    
%% Initial Condition %%    
    % Observer 1
    
%     y = C1 * x;
%     y_hat = C1 * x_hat;
%     x_dot = A*x - B * K * x_hat;
%     
%     x_hat_dot = A*x_hat - B*K*x_hat + L1*(y - y_hat);
%     x = x + x_dot * dt;
%     x_hat = x_hat + x_hat_dot * dt;
    
    % Observer 3 
    
%     y = C3 * x;
%     y_hat = C3 * x_hat;
%     x_dot = A*x - B * K * x_hat;
%     
%     x_hat_dot = A*x_hat - B*K*x_hat + L3*(y - y_hat);
%     x = x + x_dot * dt;
%     x_hat = x_hat + x_hat_dot * dt;
    
    % Observer 4
    
%     y = C4 * x;
%     y_hat = C4 * x_hat;
%     x_dot = A*x - B * K * x_hat;
%     
%     x_hat_dot = A*x_hat - B*K*x_hat + L4*(y - y_hat);
%     x = x + x_dot * dt;
%     x_hat = x_hat + x_hat_dot * dt;

%% Unit Step Input to closeloop %%
    F = 1; % Unit Input
%     % Observer 1
    
    y = C1 * x;
    y_hat = C1 * x_hat;
    x_dot = A * x + B * F - B * K * x_hat;
     
    x_hat_dot = A*x_hat + B*F - B * K * x_hat + L1*(y - y_hat);
    x = x + x_dot * dt;
    x_hat = x_hat + x_hat_dot * dt;
%     
    % Observer 3 
    
%     y = C3 * x;
%     y_hat = C3 * x_hat;
%     x_dot = A * x + B * F - B * K * x_hat;
%     
%     x_hat_dot = A*x_hat + B*F - B * K * x_hat + L3*(y - y_hat);
%     x = x + x_dot * dt;
%     x_hat = x_hat + x_hat_dot * dt;
    
    % Observer 4
     
%     y = C4 * x;
%     y_hat = C4 * x_hat;
%     x_dot = A * x + B * F - B * K * x_hat;
%     
%     x_hat_dot = A*x_hat + B*F - B * K * x_hat + L4*(y - y_hat);
%     x = x + x_dot * dt;
%     x_hat = x_hat + x_hat_dot * dt;
%% Unit Step Input to Openloop %%

%     % Observer 1
    
%     y = C1 * x;
%     y_hat = C1 * x_hat;
%     x_dot = A * x + B * F;
%      
%     x_hat_dot = A*x_hat + B*F + L1*(y - y_hat);
%     x = x + x_dot * dt;
%     x_hat = x_hat + x_hat_dot * dt;
    
    % Observer 3 
    
%     y = C3 * x;
%     y_hat = C3 * x_hat;
%     x_dot = A * x + B * F;
%     
%     x_hat_dot = A*x_hat + B*F + L3*(y - y_hat);
%     x = x + x_dot * dt;
%     x_hat = x_hat + x_hat_dot * dt;
    
    % Observer 4
     
%     y = C4 * x;
%     y_hat = C4 * x_hat;
%     x_dot = A * x + B * F;
%     
%     x_hat_dot = A*x_hat + B*F + L4*(y - y_hat);
%     x = x + x_dot * dt;
%     x_hat = x_hat + x_hat_dot * dt;

%% Record
    record_state_x(1,num) = x(1);
    record_state_x(2,num) = x(2);
    record_state_x(3,num) = x(3);
    record_state_x(4,num) = x(4);
    record_state_x(5,num) = x(5);
    record_state_x(6,num) = x(6);
    record_state_x_hat(1,num) = x_hat(1);
    record_state_x_hat(2,num) = x_hat(2);
    record_state_x_hat(3,num) = x_hat(3);
    record_state_x_hat(4,num) = x_hat(4);
    record_state_x_hat(5,num) = x_hat(5);
    record_state_x_hat(6,num) = x_hat(6);
    
    record_times(num) = t;
    
    num = num + 1;
end

%% Linear Figures %%

subplot(2,3,1);
plot(record_times,record_state_x(1,:))
title('State 1: x')
hold on;
subplot(2,3,4);
plot(record_times,record_state_x(2,:))
title('State 2: x dot')
hold on;
subplot(2,3,2);
plot(record_times,record_state_x(3,:))
title('State 3: theta1')
hold on;
subplot(2,3,5);
plot(record_times,record_state_x(4,:))
title('State 4: theta1 dot')
hold on;
subplot(2,3,3);
plot(record_times,record_state_x(5,:))
title('State 5: theta2')
hold on;
subplot(2,3,6);
plot(record_times,record_state_x(6,:))
title('State 6: theta2 dot')
hold on;

%% Observer Figure %% 

subplot(2,3,1);
plot(record_times,record_state_x_hat(1,:))
legend('State','Observer')
title('State 1: x')
subplot(2,3,4);
plot(record_times,record_state_x_hat(2,:))
legend('State','Observer')
title('State 2: x dot')
subplot(2,3,2);
plot(record_times,record_state_x_hat(3,:))
legend('State','Observer')
title('State 3: theta1')
subplot(2,3,5);
plot(record_times,record_state_x_hat(4,:))
legend('State','Observer')
title('State 4: theta1 dot')
subplot(2,3,3);
plot(record_times,record_state_x_hat(5,:))
legend('State','Observer')
title('State 5: theta2')
subplot(2,3,6);
plot(record_times,record_state_x_hat(6,:))
legend('State','Observer')
title('State 6: theta2 dot')

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



