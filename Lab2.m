clc;
clear all;
close all;
%CAN YOU SEE THIS
%% Initialization
dt = 0.1; %timestep
Tf = 15; %15 seconds
T = 0:dt:Tf; %gives 150 points
r = 0.4; %radius
w = 5; %angular velocity

%Errors
Q = [0.00335, 0.00437]'; %Measurement noise [0.00335, 0.00437]';
R = [0.05,0.05,0.01]'; %System noise %changed from 0.1 -> 0.05 for first 2 elements

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

%Storage for EKF
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));

%% Simulation
for t=2:length(T)
    x_ideal(1,t) = x_ideal(1,t-1) + v_x*dt;
    x_ideal(2,t) = x_ideal(2,t-1) + v_y*dt;
    x_ideal(3,t) = x_ideal(3,t-1) + w*dt;
    
    x(1,t) = x(1,t-1) + v_x*dt + normrnd(0,R(1));
    x(2,t) = x(2,t-1) + v_y*dt + normrnd(0,R(2));
    x(3,t) = x(3,t-1) + w*dt + normrnd(0,R(3));
    
    y(:,t) = sensor_model(x_ideal(:,t));
    y(1,t) = y(1,t) + normrnd(0,Q(1));
    y(2,t) = y(2,t) + normrnd(0,Q(2));
    
    v_x = r*w*cos(x_ideal(3,t));
    v_y = r*w*sin(x_ideal(3,t));
    
    Gt = [1, 0, -sin(x_ideal(3,t-1))/5; 
          0, 1,  cos(x_ideal(3,t-1))/5;
          0, 0,1];

    % Ali Update: looked into Ht format from Hadibi, this format matches
    % it, needed to apply jacobian for all states, not just x1,x2. Take
    % output as-is and use as Ht, no transpose needed
    
    %Ht for poly fit % not great but way better than others
    Ht = [(5142*x_ideal(1,t-1))/625 - 47153/10000,0,0;
        0,(23153*x_ideal(2,t-1))/5000 - 30747/10000,0];
    
    %Ht for Inverse fit %Terrible
%     Ht = [83741/(10000*(x_ideal(1,t-1) + 123/10000)) - ((83741*x_ideal(1,t-1))/10000 + 479/2000)/(x_ideal(1,t-1) + 123/10000)^2, 0, 0;
%             0, 41779/(5000*(x_ideal(2,t-1) + 647/5000)) - ((41779*x_ideal(2,t-1))/5000 + 834/625)/(x_ideal(2,t-1) + 647/5000)^2, 0];
    
    %Ht for power fit
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
plot(x(1,2:t),x(2,2:t), 'ro--') %state x and y (directions) for timesteps
plot(y(1,2:t), y(2,2:t), 'x--', 'Color', '#329E2B')
plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')

% figure(2)
% plot(y(1,2:t), y(2,2:t), 'x--', 'Color', '#329E2B') %measurement

% % EKF Estimate
% figure(3)
% plot(mu_S(1,2:t),mu_S(2,2:t), 'ro--') %estimate