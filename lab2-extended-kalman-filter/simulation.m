clc;
clear all;
close all;
%% Initialization
dt = 0.1; %timestep
Tf = 15; %15 seconds
T = 0:dt:Tf; %gives 150 points
r = 0.4; %radius
w = 5; %angular velocity

%Errors
%Changed to use diag. removed transpose
%TODO: use var or sigma? do we need to take squareroot?
omega_std = 0.1 * pi / 180;
R = diag([0.05,0.05,omega_std]).^2; %System noise (squared) 
Q = diag([0.00335, 0.00437]); %Measurement noise (squared)
[RE, Re] = eig (R);

% EKF Initialization
x0 = [0 0 0]'; %initial state [x,y,theta]
mu = [0 0 0]'; % mean (mu)
S = 1*eye(3);% covariance (Sigma)

% Motion and sensor init
n = length(mu);
m = 2; %values for sensor output (2 --> x,y accelerations)
x = zeros(n,length(T));
x_ideal = zeros (n, length(T));
y = zeros(m, length(T));
x(:,1) = x0;
x_ideal(:,1)= x0;

% Init velocities
v_x = r*w*cos(x(3,1));
v_y = r*w*sin(x(3,1));
v = [v_x v_y w]';

%Storage for EKF
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));

%% Simulation
for t=2:length(T)
    e = RE*sqrt(Re)*randn(n,1);
    x_ideal(:,t) = x_ideal(:,t-1) + v(:)*dt;
    x(:,t) = x(:,t-1) + e;
%     x(:,t) = x(:,t-1) + mvnrnd(zeros(1,n),R,1)';
    
    d = sqrt(Q)*randn(m,1);
    y(:,t) = sensor_model(x_ideal(:,t)) + d;
%     y(:,t) = sensor_model(x_ideal(:,t)) + mvnrnd(zeros(1,m),Q,1)';

% TODO: FIX. TRYING TO REMOVE x(3) and use arctan(x(2)/x(1)) instead
    theta = atan(x_ideal(2,t)/x_ideal(1,t));
    v(1) = r*w*cos(theta);
    v(2) = r*w*sin(theta);
    
    Gt = [1, 0, -sin(theta)/5; 
      0, 1,  cos(theta)/5;
      0, 0,1];

%     v(1) = r*w*cos(x_ideal(3,t));
%     v(2) = r*w*sin(x_ideal(3,t));
    
%     Gt = [1, 0, -sin(x_ideal(3,t-1))/5; 
%           0, 1,  cos(x_ideal(3,t-1))/5;
%           0, 0,1];
    
    %Ht (poly fit)
    Ht = [(5142*x_ideal(1,t-1))/625 - 47153/10000,0,0;
        0,(23153*x_ideal(2,t-1))/5000 - 30747/10000,0];
    
    %Ht (inverse fit)
%     Ht = [83741/(10000*(x_ideal(1,t-1) + 123/10000)) - ((83741*x_ideal(1,t-1))/10000 + 479/2000)/(x_ideal(1,t-1) + 123/10000)^2, 0, 0;
%             0, 41779/(5000*(x_ideal(2,t-1) + 647/5000)) - ((41779*x_ideal(2,t-1))/5000 + 834/625)/(x_ideal(2,t-1) + 647/5000)^2, 0];
    
    %Ht (power fit)
% 	Ht = [-25529417/(50000000*x_ideal(1,t-1)^(10619/10000)),0 , 0;
%             0, -4997949/(12500000*x_ideal(2,t-1)^(1309/1250)), 0];

    g = x_ideal(:,t); 
    Y = y(:,t);
    
    [mu,S,K,mup] = EKF(g,Gt,Ht,S,Y,@sensor_model,R,Q);
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    
end

%% Plot 
figure(1)
hold on
plot(x(1,2:t),x(2,2:t), 'ro--') %state x and y (directions) for timesteps
plot(y(1,2:t), y(2,2:t), 'x--', 'Color', '#329E2B')
plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
hold off
% figure(2)
% plot(y(1,2:t), y(2,2:t), 'x--', 'Color', '#329E2B') %measurement

% % EKF Estimate
% figure(3)
% plot(mu_S(1,2:t),mu_S(2,2:t), 'ro--') %estimate