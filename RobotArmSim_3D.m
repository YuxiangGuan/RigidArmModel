% This is a script for setting and running a 3D 2-Revolutejoint (2-R) robot simulink model

%% Define Simulation Parameters
startTime = 0;
stopTime  = 10;
stepTime = 0.1;

%% Robot Specifications
% length(m) width(m) height(m) mass(kg)
% 1         2        3         4
params_matrix = [1.0 0.2 0.2 1.5;   % Link 1
                 1.0 0.2 0.2 1.0];  % Link 2

L1 = params_matrix(1,1);
L2 = params_matrix(2,1);
m1 = params_matrix(1,4);
m2 = params_matrix(2,4);
I1 = 1/12 * m1 * (params_matrix(1,1)^2 + params_matrix(1,2)^2);
I2 = 1/12 * m2 * (params_matrix(2,1)^2 + params_matrix(2,2)^2);

init_theta_1 = -pi/4; % the initial rotation angle of joint 1 (radian)
init_theta_2 = 0; % the initial rotation angle of joint 2 (radian)
init_theta_dot_1 = 0; % initial joint 1 velocity (rad/s)
init_theta_dot_2 = 0; % initial joint 2 velocity (rad/s)

t1 = 0;
t2 = 0;

% % desired end tip position
% x_des = 1.1248;
% y_des = 1.4659;

%% Friction Parameters
t_brk = 1.0;
t_c = 0.8;
theta_dot_brk = 0.1;
f_viscous = 0.4;

%% Model Predictive Controller
k1 = 3.0;
k2 = 3.0;

%% Run the Simulation
sim('RigidArmModel_3D');