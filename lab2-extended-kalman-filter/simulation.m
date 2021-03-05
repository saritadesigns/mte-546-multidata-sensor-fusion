clc;
clear all;
close all;
%% Initialization
dt = 0.1; %timestep
Tf = 10;
T = 0:dt:Tf; 

%% Motion: STATIONARY
% %Errors
% omega_std = 0.1 * pi / 180;
% R = diag([0.05,0.05,omega_std]).^2; %System noise (squared) %OG
% Q = diag([0.00335, 0.00437]); %Measurement noise (squared) %OG
% 
% % EKF Initialization
% x0 = [0 0 0]'; %initial state [x,y,theta]
% mu = [0 0 0]'; % mean (mu)
% S = 1*eye(3);% covariance (Sigma)


%% Motion: LINE

% %Errors
% R = diag([0.05,0.05,0.05,0.05]).^2; %System noise (squared) %OG
% Q = diag([0.00335, 0.00437]); %Measurement noise (squared) %OG
% 
% % EKF Initialization
% x0 = [0 0 1 1]'; %initial state [x,y,theta]
% mu = [0 0 1 1]'; % mean (mu)
% S = 1*eye(4);% covariance (Sigma)
% A = [1 0 dt 0;0 1 0 dt;0 0 1 0;0 0 0 1];

%% Motion: CIRCLE

r = 2; %radius
w = 1; %angular velocity

%Errors
omega_std = 0.1 * pi / 180;
R = diag([0.05,0.05,omega_std]).^2; %System noise (squared) %OG
Q = diag([0.00335, 0.00437]); %Measurement noise (squared) %OG

% R = diag([0.5,0.5,omega_std]).^2; %System noise (squared) %Change
% Q = diag([0.00335, 0.00437]); %Measurement noise (squared) %Change

% EKF Initialization
x0 = [0 0 0]'; %initial state [x,y,theta]
mu = [0 0 0]'; % mean (mu)
S = 1*eye(3);% covariance (Sigma)

% Init velocities
v_x = r*w*cos(x0(3));
v_y = r*w*sin(x0(3));
v = [v_x v_y w]';

%% General init
[RE, Re] = eig (R);

% Motion and sensor init
n = length(mu);
m = 2; %values for sensor output (2 --> x,y accelerations)
x = zeros(n,length(T));
x_ideal = zeros(n, length(T));
y = zeros(m, length(T));
x(:,1) = x0;
x_ideal(:,1)= x0;

%Storage for EKF
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));
mu_S(:,1) = mu;

%% Simulation
for t=2:length(T)
    e = RE*sqrt(Re)*randn(n,1);
    d = sqrt(Q)*randn(m,1);
    
    %% Motion: STATIONARY
%     x_ideal(:,t) = 0;
%     x(:,t) = 0 + e;
%     Gt = eye(3);
%     Ht = eye(2,3);
    
    %% Motion: LINE
%     x_ideal(:,t) = A*x_ideal(:,t-1);
%     x(:,t) = A*x(:,t-1) + e;
%     Gt = eye(4);
%     Ht = eye(2,4);
    
    %% Motion: CIRLCE 
    x_ideal(:,t) = x_ideal(:,t-1) + v(:)*dt;
    x(:,t) = x(:,t-1) + v(:)*dt + e;    

    v(1) = r*w*cos(x_ideal(3,t));
    v(2) = r*w*sin(x_ideal(3,t));
    
    Gt = [1, 0, -sin(x_ideal(3,t-1))/5; 
          0, 1,  cos(x_ideal(3,t-1))/5;
          0, 0,1];
    
%     Ht (poly fit)
%     Ht = [(5142*x_ideal(1,t-1))/625 - 47153/10000,0,0;
%         0,(23153*x_ideal(2,t-1))/5000 - 30747/10000,0];
    
% %     Ht (inverse fit)
    Ht = [83741/(10000*(x_ideal(1,t-1) + 123/10000)) - ((83741*x_ideal(1,t-1))/10000 + 479/2000)/(x_ideal(1,t-1) + 123/10000)^2, 0, 0;
            0, 41779/(5000*(x_ideal(2,t-1) + 647/5000)) - ((41779*x_ideal(2,t-1))/5000 + 834/625)/(x_ideal(2,t-1) + 647/5000)^2, 0];
%     
% %     Ht (power fit)
% 	Ht = [-25529417/(50000000*x_ideal(1,t-1)^(10619/10000)),0 , 0;
%             0, -4997949/(12500000*x_ideal(2,t-1)^(1309/1250)), 0];

%% EKF
    y(:,t) = sensor_model(x_ideal(:,t)) + d;
    g = x_ideal(:,t); 
    Y = y(:,t);
    [mu,S,K,mup] = EKF(g,Gt,Ht,S,Y,@sensor_model,R,Q);
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    
end

%% Plot 
figure(1)
axis equal
hold on
plot(x_ideal(1,1:t),x_ideal(2,1:t), 'ro--') %state x and y (directions) for timesteps
plot(x(1,1:t),x(2,1:t), 'rx--') %state x and y (directions) for timesteps
plot(mu_S(1,1:t),mu_S(2,1:t), 'bx--')
legend({'actual position','noisy position','estimated position (EKF)'})
xlabel('x position')
ylabel('y position')
title('X and Y Positions (Motion)')
hold off

figure(2)
plot(y(1,2:t), y(2,2:t), 'x--', 'Color', '#329E2B')
legend({'sensor output'})
xlabel('x acceleration')
ylabel('y acceleration')
title('Sensor Model Output')

