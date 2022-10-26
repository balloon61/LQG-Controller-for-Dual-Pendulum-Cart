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
     0 0 100 0 0 0;
     0 0 0 100 0 0;
     0 0 0 0 100 0;
     0 0 0 0 0 100];
 
 
 
x = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1];
non_x = x;
dt = 0.001;
Times = 1000/dt; 
record_statex = zeros(6,Times);
record_non_x = zeros(6,Times);
record_times = zeros(1,Times);

K = lqr(A,B,Q,R);
num = 1;

%% Simulation %%
for t = 0.0 : dt : Times*dt
    
    % Observer 1
    
    x_dot = A*x - B*K*x;
    non_x_A = [non_x(2); 
        (-m1*g*sin(non_x(3))*cos(non_x(3))-m1*l1*non_x(4)*non_x(4)*sin(non_x(3)) - m2*g*sin(non_x(5))*cos(non_x(5)) - m2*l2*non_x(6)*non_x(6)*sin(non_x(5))) / (m1*sin(non_x(3))*sin(non_x(3)) + m2*sin(non_x(5))*sin(non_x(5))+M);
        non_x(4);
        (-m1*g*sin(non_x(3))*cos(non_x(3))-m1*l1*non_x(4)*non_x(4)*sin(non_x(3)) - m2*g*sin(non_x(5))*cos(non_x(5)) - m2*l2*non_x(6)*non_x(6)*sin(non_x(5))) * cos(non_x(3)) / (l1 * (m1*sin(non_x(3))*sin(non_x(3)) + m2*sin(non_x(5))*sin(non_x(5))+M)) - g*sin(non_x(3))/l1;
        non_x(6);
        (-m1*g*sin(non_x(3))*cos(non_x(3))-m1*l1*non_x(4)*non_x(4)*sin(non_x(3)) - m2*g*sin(non_x(5))*cos(non_x(5)) - m2*l2*non_x(6)*non_x(6)*sin(non_x(5))) * cos(non_x(5)) / (l2 * (m1*sin(non_x(3))*sin(non_x(3)) + m2*sin(non_x(5))*sin(non_x(5))+M)) - g*sin(non_x(5))/l2];
    non_x_B = [0;
               1/(M + m1 * sin(non_x(3)) * sin(non_x(3)));
               0;
               cos(non_x(3))/(l1*(M + m1 * sin(non_x(3)) * sin(non_x(3)) + m2 * sin(non_x(5)) * sin(non_x(5))));
               0;
               cos(non_x(5))/(l2*(M + m1 * sin(non_x(3)) * sin(non_x(3)) + m2 * sin(non_x(5)) * sin(non_x(5))))];
    non_x_dot = non_x_A - non_x_B * K * x; 
    x = x + x_dot * dt;
    non_x = non_x + non_x_dot * dt;


%% Record
    record_statex(1,num) = x(1);
    record_statex(2,num) = x(2);
    record_statex(3,num) = x(3);
    record_statex(4,num) = x(4);
    record_statex(5,num) = x(5);
    record_statex(6,num) = x(6);
    
    record_non_x(1,num) = non_x(1);
    record_non_x(2,num) = non_x(2);
    record_non_x(3,num) = non_x(3);
    record_non_x(4,num) = non_x(4);
    record_non_x(5,num) = non_x(5);
    record_non_x(6,num) = non_x(6);
    
    record_times(num) = t;
    
    num = num + 1;
end



%% Linear Figures %%

subplot(2,3,1);
plot(record_times,record_statex(1,:))
title('State 1: x')
hold on;
subplot(2,3,4);
plot(record_times,record_statex(2,:))
title('State 2: x dot')
hold on;
subplot(2,3,2);
plot(record_times,record_statex(3,:))
title('State 3: theta1')
hold on;
subplot(2,3,5);
plot(record_times,record_statex(4,:))
title('State 4: theta1 dot')
hold on;
subplot(2,3,3);
plot(record_times,record_statex(5,:))
title('State 5: theta2')
hold on;
subplot(2,3,6);
plot(record_times,record_statex(6,:))
title('State 6: theta2 dot')
hold on;
 
%% Nonlinear Figure %%
 
subplot(2,3,1);
plot(record_times,record_non_x(1,:))
title('State 1: x')
subplot(2,3,4);
plot(record_times,record_non_x(2,:))
title('State 2: x dot')
subplot(2,3,2);
plot(record_times,record_non_x(3,:))
title('State 3: theta1')
subplot(2,3,5);
plot(record_times,record_non_x(4,:))
title('State 4: theta1 dot')
subplot(2,3,3);
plot(record_times,record_non_x(5,:))
title('State 5: theta2')
subplot(2,3,6);
plot(record_times,record_non_x(6,:))
title('State 6: theta2 dot') 
 
 
 
 %%
 
 

%  function dydt = vdp1(t,y)
% x1_dot = x2;
% x2_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) / (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M);
% x3_dot = x4;
% x4_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) /(l1 * (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M)) - g*sin(x3)/l1;
% x5_dot = x6;
% x6_dot = (-m1*g*sin(x3)*cos(x3)-m1*l1*x4*x4*sin(x3) - -m2*g*sin(x5)*cos(x5)-m2*l2*x6*x6*sin(x5)) /(l2 * (m1*sin(x3)*sin(x3) + m2*sin(x5)*sin(x5)+M)) - g*sin(x5)/l2;

% dydt = [y(2); (1-y(1)^2)*y(2)-y(1)];
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 