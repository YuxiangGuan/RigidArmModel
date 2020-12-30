% This is a script for setting and running a planar 2-Revolutejoint (2-R) robot simulink model

% Assumptions: (1)The two link masses are point masses; 

%% Define Simulation Parameters
startTime = 0;
stopTime  = 20;
stepTime = 0.1;

%% Robot Specifications
L1 = 1; % the length of link 1 (m)
L2 = 1; % the length of link 2 (m)
m1 = 1; % the mass of link 1 (kg)
m2 = 1; % the mass of link 2 (kg)
init_theta_1 = 0; % the initial rotation angle of joint 1 (radian)
init_theta_2 = 0; % the initial rotation angle of joint 2 (radian)
% t1 = 30.3;
% t2 = 10.2;
init_theta_dot_1 = 0; % initial joint 1 velocity (rad/s)
init_theta_dot_2 = 0; % initial joint 2 velocity (rad/s)

% desired end tip position
x_des = 1.1248;
y_des = 1.4659;

%% Model Predictive Controller
k1 = 3.0;
k2 = 3.0;

%% Run the Simulation
sim('RigidArmModel_2D');