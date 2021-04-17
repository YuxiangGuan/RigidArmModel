% This is a script for setting and running a soft robotic manipulator simulink model

%% Robot Specifications
% m: link mass; l: link length at relaxation; d: daming; k: stiffness; 
m = [0.01; 0.01];
l = [0.2; 0.2];
d = [2.0; 2.0];
k = [0.5; 0.5];

%% Initial Conditions
q_init = [1e-6; 1e-6];    % rad
dq_init = [1e-6; 1e-6];   % rad/s  

%% Trajectory Following
rng default
gs = GlobalSearch;
l1 = l(1);
l2 = l(2);
tracking_method = 1;

if tracking_method == 1
    % Single point tracking
    pd = [0.3 0.2]; % Desired manipulator end-tip position
    q_dt = zeros(size(pd,1),size(pd,2));
    q_gmin = @(q)(norm([(l(1)*sin(q(1)))/q(1) + (l(2)*cos(q(1))*sin(q(2)))/q(2) + (l(2)*sin(q(1))*(cos(q(2)) - 1))/q(2) - pd(1,1);(l(2)*sin(q(1))*sin(q(2)))/q(2) - (l(1)*(cos(q(1)) - 1))/q(1) - (l(2)*cos(q(1))*(cos(q(2)) - 1))/q(2) - pd(1,2)]) / ...
                norm(pd(1,:)));
    problem = createOptimProblem('fmincon','x0',[1e-3,1e-3],'objective',q_gmin,'lb',[-pi,-pi],'ub',[pi,pi]);
    [q_min,fval] = run(gs, problem);
    q_dt(1,:) = q_min;
    
    q_d.time = [];
    q_d.signals.values = [q_dt(1,1),q_dt(1,2)];
    q_d.signals.dimensions = 2;
elseif tracking_method == 2
    % Serial points tracking (need to change q_d and adjust sim time in Simulink)
    pd = zeros(2000,2);
    for i = 1:2000
        pd(i,1) = 0.3 - 0.0001 * i;
        pd(i,2) = 0.2;
    end
    q_dt = zeros(size(pd,1),size(pd,2));
    q_gmin = @(q)(norm([(l(1)*sin(q(1)))/q(1) + (l(2)*cos(q(1))*sin(q(2)))/q(2) + (l(2)*sin(q(1))*(cos(q(2)) - 1))/q(2) - pd(1,1);(l(2)*sin(q(1))*sin(q(2)))/q(2) - (l(1)*(cos(q(1)) - 1))/q(1) - (l(2)*cos(q(1))*(cos(q(2)) - 1))/q(2) - pd(1,2)]) / ...
                norm(pd(1,:)));
    problem = createOptimProblem('fmincon','x0',[1e-3,1e-3],'objective',q_gmin,'lb',[-pi,-pi],'ub',[pi,pi]);
    [q_min,fval] = run(gs, problem);
    q_dt(1,:) = q_min;

    for i = 2:size(pd,1)
        q_lmin = @(q)(norm([(l(1)*sin(q(1)))/q(1) + (l(2)*cos(q(1))*sin(q(2)))/q(2) + (l(2)*sin(q(1))*(cos(q(2)) - 1))/q(2) - pd(i,1);(l(2)*sin(q(1))*sin(q(2)))/q(2) - (l(1)*(cos(q(1)) - 1))/q(1) - (l(2)*cos(q(1))*(cos(q(2)) - 1))/q(2) - pd(i,2)]) / ...
                norm(pd(i,:)));
        q0 = q_min;
        q_min = fminsearch(q_lmin, q0);
        q_dt(i,:) = q_min;
    end

    q_dtt = zeros(2200,2);
    q_dtt(1:201,1) = q_dt(1,1);
    q_dtt(1:201,2) = q_dt(1,2);
    q_dtt(202:2200,1) = q_dt(2:2000,1);
    q_dtt(202:2200,2) = q_dt(2:2000,2);

    t = 0.1 * [0:2199]'; 
    q1_d = q_dtt(:,1) + t * 1e-10;
    q2_d = q_dtt(:,2) + t * 1e-10;
    q_d.time = [];
    q_d.signals.values = [q1_d,q2_d];
    q_d.signals.dimensions = 2;
end

%% Run the Simulation
sim_time = 25;
q_out = sim('SoftManipulatorModel', 'StopTime', num2str(sim_time));
q = q_out.get('q');
assignin('base', 'q', q);

%% Plot Manipulator End Tip Trajectory
r = zeros(2,1); % Arc's radius
c = zeros(2,2); % Arc's center
end_point = zeros(2,3);
end_tip = zeros(1e5,2);

if tracking_method == 1
    step_size = 1;
else
    step_size = 10;
end

k = 1;

for i = 1:step_size:size(q,1)
    r = l ./ [abs(q(i,1)); abs(q(i,2))];
    T_01 = [cos(q(i,1)) -sin(q(i,1)) l(1) * sin(q(i,1)) / q(i,1);
            sin(q(i,1)) cos(q(i,1))  l(1) * (1 - cos(q(i,1))) / q(i,1);
            0           0            1];
        
    end_point(1,1) = 0;
    end_point(2,1) = 0;
    end_point(1,2) = l(1) * sin(q(i,1)) / q(i,1);
    end_point(2,2) = l(1) * (1 - cos(q(i,1))) / q(i,1);
    end_point(1,3) = l(1) * sin(q(i,1)) / q(i,1) + l(2) * cos(q(i,1)) * sin(q(i,2)) / q(i,2) - l(2) * sin(q(i,1)) * (1 - cos(q(i,2))) / q(i,2);
    end_point(2,3) = l(1) * (1 - cos(q(i,1))) / q(i,1) + l(2) * sin(q(i,1)) * sin(q(i,2)) / q(i,2) + l(2) * cos(q(i,1)) * (1 - cos(q(i,2))) / q(i,2);
    
    end_tip(k,1) = end_point(1,3);
    end_tip(k,2) = end_point(2,3);
    k = k + 1;
    
    if q(i,1) < 0
        c(2,1) = -r(1);
    elseif q(i,1) > 0
        c(2,1) = r(1);
    end
    
    if q(i,2) < 0
        c(2,2) = -r(2);
    elseif q(i,2) > 0
        c(2,2) = r(2);
    end
    
    c(1,1) = 0;
    c(1,2) = 0;
    c_tem = T_01 * [c(1,2); c(2,2); 1];
    c(1,2) = c_tem(1);
    c(2,2) = c_tem(2);
    
    a1 = atan2(end_point(2,1) - c(2,1), end_point(1,1) - c(1,1));
    b1 = atan2(end_point(2,2) - c(2,1), end_point(1,2) - c(1,1));
    t1 = linspace(a1, b1);
    x1 = c(1,1) + r(1) * cos(t1);
    y1 = c(2,1) + r(1) * sin(t1);
    
    a2 = atan2(end_point(2,2) - c(2,2), end_point(1,2) - c(1,2));
    b2 = atan2(end_point(2,3) - c(2,2), end_point(1,3) - c(1,2));
    if abs(a2 - b2) > pi
        a2 = a2 + 2 * pi;
    end 
    t2 = linspace(a2, b2);
    x2 = c(1,2) + r(2) * cos(t2);
    y2 = c(2,2) + r(2) * sin(t2);
    
    plot([x1 x2], [y1 y2], 'r-', 'LineWidth', 1.5)
    grid on
    axis([0 0.5 -0.25 0.25])
    axis equal
    pause(0.01);
end

hold on
plot(end_tip(1:k-1,1),end_tip(1:k-1,2), 'ro')
hold off
