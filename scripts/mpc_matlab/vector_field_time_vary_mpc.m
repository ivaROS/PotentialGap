clc;
clear;
close all;
global obs_obj_1;
global obs_obj_2;
global theta;
global global_goal;
theta = pi/4;
obs_obj_1 = [9.5, 16, 0] / 5;
obs_obj_2 = [-3.25, 20.8, 0]/ 5;
global_goal = [6.5, 21, 0] / 5;
% global_goal = [-0.5, 5.5, 0];
rbt_pose = [0, 0, 0];

obs_radius = 0.2;
rbt_radius = 0.18;
f = figure;

% Plot obstacles and goal
draw_circle(obs_obj_1(1),obs_obj_1(2),obs_radius, 1, 'b');
draw_circle(obs_obj_2(1),obs_obj_2(2),obs_radius, 1, 'b');
draw_circle(global_goal(1),global_goal(2),obs_radius, 2, 'g');

% Plot robot triangle
rbt_x = [-0.3, 0.3, 0, -0.3];
rbt_y = [-0.1, -0.1, 0.4, -0.1];
plot(rbt_x, rbt_y, 'r', 'LineWidth',2);
hold on;

times = linspace(0, 30);

r = 25;
c = 25;

x = linspace(-4,4,r);
y = linspace(-1,6,c);
px = zeros(r,c);
py = zeros(r,c);

% Compute vector field
for i = 1:r
    for j = 1:c
        coor = [0;0];
        coor(1) = x(i);
        coor(2) = y(j);
        dev = polar_derivative(0, coor);
%         dev = circular_polar_derivative(0, coor);
        px(j,i) = dev(1);
        py(j,i) = dev(2);
    end
end

% Plot boundaries of potential field and field vectors
theta1 = atan2(obs_obj_1(2), obs_obj_1(1));
theta2 = atan2(obs_obj_2(2), obs_obj_2(1));
r1 = sqrt(sum(obs_obj_1.^2));
r2 = sqrt(sum(obs_obj_2.^2));

plot_t = linspace(theta1, theta2);
plot_r = (plot_t - theta1) ./ (theta2 - theta1) * (r2 - r1) + r1;
arc_x = plot_r .* cos(plot_t);
arc_y = plot_r .* sin(plot_t);
plot(arc_x, arc_y, 'r', 'LineWidth', 2);
hold on;

quiver(x, y, px, py, 'k');
hold on;
plot([obs_obj_2(1), 0], [obs_obj_2(2), 0], 'r', 'LineWidth', 2);
hold on;
plot([obs_obj_1(1), 0], [obs_obj_1(2), 0], 'r', 'LineWidth', 2);
axis off;

% Get the trajectory and plot
use_potential_field = false;

if use_potential_field
    % [t,w] = ode45(@(t, y) derivative(t, y), times, [0,0]);
    [t,w] = ode45(@(t, y) polar_derivative(t, y), times, [0,0]);
else
    [t,w] = mpc_pg(times, [0,0]);
end


[end_size, ~] = size(w);
for i = 1:end_size
    hold on;
    plot(w(i,1),w(i,2), 'o', 'LineWidth', 2, 'MarkerSize', 8);
end
axis off;
axis equal;

% % Plot cost values
% % gap cost
% y_init = [0,0];
% vec_1 = obs_obj_1(1:2) - y_init;
% normal_1 = [1, -vec_1(1) / vec_1(2)];
% normal_1 = normal_1 / norm(normal_1);
% vec_2 = obs_obj_2(1:2) - y_init;
% normal_2 = [-1, vec_2(1) / vec_2(2)];
% normal_2 = normal_2 / norm(normal_2);
% [x_mat, y_mat] = meshgrid(x, y);
% xy_length = sqrt(x_mat.^2 + y_mat.^2) + 1e-8;
% xy_obs_dist_1 = sqrt((x_mat - obs_obj_1(1)).^2 + (y_mat - obs_obj_1(2)).^2);
% gap_cost_1 = (x_mat * normal_1(1) + y_mat * normal_1(2)) ./ xy_length .* exp(-xy_obs_dist_1);
% xy_obs_dist_2 = sqrt((x_mat - obs_obj_2(1)).^2 + (y_mat - obs_obj_2(2)).^2);
% gap_cost_2 = (x_mat * normal_2(1) + y_mat * normal_2(2)) ./ xy_length .* exp(-xy_obs_dist_2);
% gap_cost = gap_cost_1 + gap_cost_2;
% % gap_cost = zeros(size(x, 2), size(y, 2));
% % for i = 1:size(x, 2)
% %     for j = 1:size(y, 2)
% %         x_vec = [x(i), y(j)] - y_init;
% % %         gap_cost(i, j) = x_vec * normal_1' + x_vec * normal_2';
% %         gap_cost(i, j) = abs(x_vec * normal_1');
% %     end
% % end
% 
% figure(2)
% surf(x_mat, y_mat, gap_cost);
% view(2); 
% % pcolor(x_mat, y_mat, gap_cost_1);
% 
% % goal cost
% goal_cost = sqrt((x_mat - global_goal(1)).^2 + (y_mat - global_goal(2)).^2);
% % goal_cost = zeros(size(x, 2), size(y, 2));
% % for i = 1:size(x, 2)
% %     for j = 1:size(y, 2)
% %         goal_dist = [x(i), y(j)] - global_goal(1:2);
% %         goal_cost(i, j) = norm(goal_dist);
% %     end
% % end
% 
% figure(3)
% surf(x_mat, y_mat, goal_cost);
% view(2); 
% % pcolor(x_mat, y_mat, goal_cost);
% 
% figure(4)
% total_cost = gap_cost + 5e-2 * goal_cost;
% surf(x_mat, y_mat, total_cost);
% view(2); 
% % pcolor(x_mat, y_mat, gap_cost + 1e-1 * goal_cost);

% 

function derivs = polar_derivative(t, y)

    global theta;
    global obs_obj_1;
    global obs_obj_2;
    global global_goal;
    obs1 = obs_obj_1;
    obs2 = obs_obj_2;
    theta_local = theta;
    
    vec_1 = obs1(1:2) - y';
    
%     angle1 = atan2(obs1(2), obs1(1));
%     angle2 = atan2(obs2(2), obs2(1));
%     anglex = atan2(y(2), y(1));
    
    angle1 = obs1(2) * 1i + obs1(1);
    anglex = y(2) * 1i + y(1);
    diff1 = angle(angle1 / anglex);
    
    
    angle2 = obs2(2) * 1i + obs2(1);
    diff2 = angle(angle2 / anglex);
    
%     if anglex < 0
%         anglex = anglex + 2 * pi;
%     end
%     
%     if angle1 < 0
%         angle1 = angle1 + 2 * pi;
%     end
%     diff1 = angle1 - anglex;
    
    vec_1 = vec_1 / norm(vec_1) * exp(- abs(diff1) ); 
    vec_1 = R(pi/2) * vec_1';
    c1 = vec_1;
    
    vec_2 = obs2(1:2) - y';
    vec_2 = vec_2 / norm(vec_2) * exp(- abs(diff2)); 
    vec_2 = R(-pi/2) * vec_2';
    c2 = vec_2;
    
    theta1 = atan2(obs1(2), obs1(1));
    theta2 = atan2(obs2(2), obs2(1));
    thetag = atan2(global_goal(2) - y(2), global_goal(1) - y(1));
    thetax = atan2(y(2), y(1));
    r1 = norm(obs1(1:2));
    r2 = norm(obs2(1:2));
    rx = norm(y);
    
    thetalim = thetag;
    if thetag > theta2
        thetalim = theta2;
    elseif thetag < theta1
        thetalim = theta1;
    else
    end
    
    k = (thetax - theta1) / (theta2 - theta1) * (r2 - r1) + r1;
    if (rx > k)
        res_t = thetag;
%         c1 = c1 * 0.0;
%         c2 = c2 * 0.0;
    else
        res_t = thetalim;
    end
    
    c3 = [cos(res_t); sin(res_t)];
%     dist1 = norm(obs1(1:2) - y');
%     dist2 = norm(obs2(1:2) - y');
%     if dist1 > dist2
%         close_pt = obs2(1:2);
%         far_pt = obs1(1:2);
%     else 
%         close_pt = obs1(1:2);
%         far_pt = obs2(1:2);
%     end
%     
%     far_vec = far_pt' - close_pt';
%     rbt_vec =  y - close_pt';
% 
%     angle_gap = acos(sum(far_vec .* rbt_vec) / (norm(far_vec) * norm(rbt_vec)));
%     if angle_gap > pi/2
%         dir_vec = rbt_vec;
%     else
%         dir_vec = R(-pi/2) * (obs1(1:2)' - obs2(1:2)');
%     end
%     dir_vec = - dir_vec / norm(dir_vec);
% 
%     c3 = [sin(angle_gap) * norm(dir_vec)]^(-1) * dir_vec;
%     c3 = dir_vec;
    derivs = c1 * 1 + c2 * 1 + c3 * 1;
%     derivs = c3 * 0;
end

function derivs = polar_derivative_t(t, y)

    global theta;
    global obs_obj_1;
    global obs_obj_2;
    global global_goal;
    obs1 = obs_obj_1(1:2) - y';
    obs2 = obs_obj_2(1:2) - y';
    goal_local = global_goal(1:2) - y';
    theta_local = theta;
    
    vec_1 = obs1;
    
%     angle1 = atan2(obs1(2), obs1(1));
%     angle2 = atan2(obs2(2), obs2(1));
%     anglex = atan2(y(2), y(1));
    
    angle1 = obs1(2) * 1i + obs1(1);
%     anglex = 0 * 1i + 0;
    diff1 = angle(angle1);
    
    
    angle2 = obs2(2) * 1i + obs2(1);
    diff2 = angle(angle2);
    
%     if anglex < 0
%         anglex = anglex + 2 * pi;
%     end
%     
%     if angle1 < 0
%         angle1 = angle1 + 2 * pi;
%     end
%     diff1 = angle1 - anglex;
    
    vec_1 = vec_1 / norm(vec_1) * exp(- abs(diff1) ); 
    vec_1 = R(pi/2) * vec_1';
    c1 = vec_1;
    
    vec_2 = obs2;
    vec_2 = vec_2 / norm(vec_2) * exp(- abs(diff2)); 
    vec_2 = R(-pi/2) * vec_2';
    c2 = vec_2;
    
    theta1 = atan2(obs1(2), obs1(1));
    theta2 = atan2(obs2(2), obs2(1));
    thetag = atan2(goal_local(2), goal_local(1));
    thetax = 0;
    r1 = norm(obs1(1:2));
    r2 = norm(obs2(1:2));
    rx = 0;
    
    thetalim = thetag;
    if thetag > theta2
        thetalim = theta2;
    elseif thetag < theta1
        thetalim = theta1;
    else
    end
    
    k = (thetax - theta1) / (theta2 - theta1) * (r2 - r1) + r1;
    if (rx > k)
        res_t = thetag;
%         c1 = c1 * 0.0;
%         c2 = c2 * 0.0;
    else
        res_t = thetalim;
    end
    
    c3 = [cos(res_t); sin(res_t)];
%     dist1 = norm(obs1(1:2) - y');
%     dist2 = norm(obs2(1:2) - y');
%     if dist1 > dist2
%         close_pt = obs2(1:2);
%         far_pt = obs1(1:2);
%     else 
%         close_pt = obs1(1:2);
%         far_pt = obs2(1:2);
%     end
%     
%     far_vec = far_pt' - close_pt';
%     rbt_vec =  y - close_pt';
% 
%     angle_gap = acos(sum(far_vec .* rbt_vec) / (norm(far_vec) * norm(rbt_vec)));
%     if angle_gap > pi/2
%         dir_vec = rbt_vec;
%     else
%         dir_vec = R(-pi/2) * (obs1(1:2)' - obs2(1:2)');
%     end
%     dir_vec = - dir_vec / norm(dir_vec);
% 
%     c3 = [sin(angle_gap) * norm(dir_vec)]^(-1) * dir_vec;
%     c3 = dir_vec;
    derivs = c1 * 1 + c2 * 1 + c3 * 1;
%     derivs = c3 * 0;
end

function derivs = circular_polar_derivative(t, y)

    global theta;
    global obs_obj_1;
    global obs_obj_2;
    global global_goal;
    obs1 = obs_obj_1;
    obs2 = obs_obj_2;
    theta_local = theta;
    
    vec_1 = obs1(1:2) - y';
    
%     angle1 = atan2(obs1(2), obs1(1));
%     angle2 = atan2(obs2(2), obs2(1));
%     anglex = atan2(y(2), y(1));
    
    angle1 = obs1(2) * 1i + obs1(1);
    anglex = y(2) * 1i + y(1);
    diff1 = angle(angle1 / anglex);
    
    
    angle2 = obs2(2) * 1i + obs2(1);
    diff2 = angle(angle2 / anglex);
    
%     if anglex < 0
%         anglex = anglex + 2 * pi;
%     end
%     
%     if angle1 < 0
%         angle1 = angle1 + 2 * pi;
%     end
%     diff1 = angle1 - anglex;
    
    vec_1 = vec_1 / norm(vec_1) * exp(- abs(diff1) ); 
    vec_1 = R(pi/2) * vec_1';
    c1 = vec_1;
    
    vec_2 = obs2(1:2) - y';
    vec_2 = vec_2 / norm(vec_2) * exp(- abs(diff2)); 
    vec_2 = R(-pi/2) * vec_2';
    c2 = vec_2;
    
    theta1 = atan2(obs1(2), obs1(1));
    theta2 = atan2(obs2(2), obs2(1));
    thetag = atan2(global_goal(2) - y(2), global_goal(1) - y(1));
    thetax = atan2(y(2), y(1));
    r1 = norm(obs1(1:2));
    r2 = norm(obs2(1:2));
    rx = norm(y);
    
    thetalim = thetag;
    if thetag > theta2
        thetalim = theta2;
    elseif thetag < theta1
        thetalim = theta1;
    else
    end
    
    k = (thetax - theta1) / (theta2 - theta1) * (r2 - r1) + r1;
    if (rx > k)
        res_t = thetag;
%         c1 = c1 * 0.0;
%         c2 = c2 * 0.0;
    else
        res_t = thetalim;
    end
    
    c3 = [cos(res_t); sin(res_t)];
%     dist1 = norm(obs1(1:2) - y');
%     dist2 = norm(obs2(1:2) - y');
%     if dist1 > dist2
%         close_pt = obs2(1:2);
%         far_pt = obs1(1:2);
%     else 
%         close_pt = obs1(1:2);
%         far_pt = obs2(1:2);
%     end
%     
%     far_vec = far_pt' - close_pt';
%     rbt_vec =  y - close_pt';
% 
%     angle_gap = acos(sum(far_vec .* rbt_vec) / (norm(far_vec) * norm(rbt_vec)));
%     if angle_gap > pi/2
%         dir_vec = rbt_vec;
%     else
%         dir_vec = R(-pi/2) * (obs1(1:2)' - obs2(1:2)');
%     end
%     dir_vec = - dir_vec / norm(dir_vec);
% 
%     c3 = [sin(angle_gap) * norm(dir_vec)]^(-1) * dir_vec;
%     c3 = dir_vec;
%     derivs = c1 * 1 + c2 * 1 + c3 * 1;
    derivs = c1 * 1 + c2 * 1;
%     derivs = c3 * 0;
end

function derivs = derivative(t, y)
    disp(y)
    global theta;
    global obs_obj_1;
    global obs_obj_2;
    global global_goal;
    obs1 = obs_obj_1;
    obs2 = obs_obj_2;
    theta_local = theta;
    
    vec_1 = obs1(1:2) - y';
    vec_1 = vec_1 / norm(vec_1) / (max([0.5, norm(vec_1)]));
    vec_1 = R(pi/2) * vec_1';
    c1 = vec_1;
    
    vec_2 = obs2(1:2) - y';
    vec_2 = vec_2 / norm(vec_2) / max([0.5, norm(vec_2)]);
    vec_2 = R(-pi/2) * vec_2';
    c2 = vec_2;

    dist1 = norm(obs1(1:2) - y');
    dist2 = norm(obs2(1:2) - y');
    if dist1 > dist2
        close_pt = obs2(1:2);
        far_pt = obs1(1:2);
    else 
        close_pt = obs1(1:2);
        far_pt = obs2(1:2);
    end
    
    far_vec = far_pt' - close_pt';
    rbt_vec =  y - close_pt';

    angle_gap = acos(sum(far_vec .* rbt_vec) / (norm(far_vec) * norm(rbt_vec)));
    if angle_gap > pi/2
        dir_vec = rbt_vec;
    else
        dir_vec = R(-pi/2) * (obs1(1:2)' - obs2(1:2)');
    end
    dir_vec = - dir_vec / norm(dir_vec);
    c3 = dir_vec;
    
%     goal_pt = [2;7];
    goal_pt = global_goal(1:2)';
    goal_vec = goal_pt - y;
    
    polar_vec = y/norm(y);
    if any(isnan(polar_vec))
        polar_vec = zeros(2,1);
    end
    
    derivs = c1 * 1 + c2 * 1 + c3 * 0 + 1 * polar_vec + goal_vec / norm(goal_vec);
%     derivs = rbt_vec;

end

function j = vector_heading_cost(input_x, use_goal_vec)
    j = 0;
    
    for i = 1:size(input_x, 1)
        if use_goal_vec
            pg_vec = polar_derivative_t(0, input_x(i, 1:2)');
        else
            pg_vec = circular_polar_derivative(0, input_x(i, 1:2)');
        end
        
        vec_angle = atan2(pg_vec(2), pg_vec(1));
        angle_diff = abs(vec_angle - input_x(i, 3));
        j = j + angle_diff;
    end
end

function [t, w] = mpc_pg(times, y_init)
    global theta;
    global obs_obj_1;
    global obs_obj_2;
    global global_goal;
    obs1 = obs_obj_1;
    obs2 = obs_obj_2;
    theta_local = theta;
    goal = global_goal;
    
    vec_1 = obs1(1:2) - y_init;
    normal_1 = [1, -vec_1(1) / vec_1(2)];
    normal_1 = normal_1 / norm(normal_1);
    vec_2 = obs2(1:2) - y_init;
    normal_2 = [-1, vec_2(1) / vec_2(2)];
    normal_2 = normal_2 / norm(normal_2);
    
    theta1 = atan2(obs1(2), obs1(1));
    theta2 = atan2(obs2(2), obs2(1));
    
    ivalab.loadLibraries( {'control','figutils'} );
    ivalab.checkPackages( {'Lie','curve'} );
    
    %==[3] Track using MPC pipeline
    x0 = [y_init'; 0];
    u0 = [0; 0];
    
    system.Ts = 0.1;
    
    nuEq = 1;

    metaBuilder1 = struct();
      %! Time horizons.
    metaBuilder1.dt.predict = 6;
    metaBuilder1.dt.control = 6;
      %! Velocity limits.
    metaBuilder1.MV(1).min = nuEq*0.0;  % m/s, forward
    metaBuilder1.MV(1).max = nuEq*0.5;  % m/s, forward
    metaBuilder1.MV(2).min = -pi/3;     % rad/s, turn
    metaBuilder1.MV(2).max =  pi/3;     % rad/s, turn

      %! Instantiate.
    ddTraj = trajSynth.mpcDiffDrive(system, metaBuilder1);
    ddTraj.setSolPackage(trajSynth.mpcDiffDrive.pTSE2);
    
    ph = ddTraj.minst.PredictionHorizon;
    goal_matrix = ones(ph, 2);
%     goal_matrix = ones(1, 2);
    goal_matrix(:, 1) = goal(1) * goal_matrix(:, 1);
    goal_matrix(:, 2) = goal(2) * goal_matrix(:, 2);
    obs_1_matrix = ones(ph, 2);
%     obs_1_matrix = ones(1, 2);
    obs_1_matrix(:, 1) = obs1(1) * obs_1_matrix(:, 1);
    obs_1_matrix(:, 2) = obs1(2) * obs_1_matrix(:, 2);
    obs_2_matrix = ones(ph, 2);
%     obs_2_matrix = ones(1, 2);
    obs_2_matrix(:, 1) = obs2(1) * obs_2_matrix(:, 1);
    obs_2_matrix(:, 2) = obs2(2) * obs_2_matrix(:, 2);
    goal_weight = 1e1;
    ddTraj.minst.Optimization.CustomCostFcn = @(X,U,e,data,params) vector_heading_cost(X(2:end, :), true) +...
                                                                   goal_weight * sum(sqrt(sum((X(2:end, 1:2) - goal_matrix).^2, 2)));
%     ddTraj.minst.Optimization.CustomCostFcn = @(X,U,e,data,params) vector_heading_cost(X(2:end, :), false) +...
%                                                                    goal_weight * sum(sqrt(sum((X(2:end, 1:2) - goal_matrix).^2, 2)));
%     ddTraj.minst.Optimization.CustomCostFcn = @(X,U,e,data,params) goal_weight * sum(sqrt(sum((X(2:end, 1:2) - goal_matrix).^2, 2)));
%     ddTraj.minst.Optimization.CustomCostFcn = @(X,U,e,data,params) vector_heading_cost(X(2:end, :));
    ddTraj.minst.Optimization.CustomIneqConFcn = @(X,U,e,data,params) [atan2(X(2:end, 2), X(2:end, 1)) - theta1; theta2 - atan2(X(2:end, 2), X(2:end, 1));...
                                                                       metaBuilder1.MV(1).min - U(1:end-1, 1); U(1:end-1, 1) - metaBuilder1.MV(1).max;...
                                                                       metaBuilder1.MV(2).min - U(1:end-1, 2); U(1:end-1, 2) - metaBuilder1.MV(2).max;...
                                                                       ];
    ddTraj.minst.Optimization.ReplaceStandardCost = true;
%     ddTraj.minst.Optimization.CustomEqConFcn = @(X,U,data,params) sqrt(sum((X(end, 1:2)' - goal(1:2)).^2, 2));

    istate.x = x0;
    istate.u = u0;

    %make an array dealing tspan from 0 to ts
    %  myTime = [desTraj.tspan(1):this.Ts:desTraj.tspan(2)];
    %  myspan = desTraj.tspan;
    %  desTraj = desTraj.xt.f(myTime);
    %  desTraj = desTraj(1:2,:);
    %
    %  tlast = myspan(1);
    %  u0 = istate.u;
    %
    %  path = desTraj(:,2:end);     %! First one can't be met, not part of MPC.

    %  tpath = transpose(path);

    %this.tauClose = 0.5;  % Later on, how specified. System or metaBuilder?

    %!--[1] Process the initial state information.
    if (isstruct(istate))
        xNow = istate.x;
        xNow = xNow(1:3);       % Should be put elsewhere. state2vec?
    else
        xNow = istate;
    end

    if (isfield(istate, 'u'))
        uNow = istate.u;
    else
        uNow = [0;0];
    end

    %!--[2] NL MPC setup
    options = nlmpcmoveopt;
    options.Parameters = {ddTraj.Ts};

    validateFcns(ddTraj.minst, xNow, uNow, [], {ddTraj.Ts});

    %!--[3] Iterate over full horizon time segments.
    %!
    ci = 1;   % @todo May not be needed. Eventually need to recode function.

    done = false;

    dSol.t = times(1);
    dSol.x = xNow;
    dSol.y = xNow(1:2);
    dSol.u = [];
    
    t = times(1);
    w = y_init;

    numIntervals = ceil((times(end) - times(1)) / ...
                                       (ddTraj.minst.PredictionHorizon*ddTraj.Ts));

    tSteps = [1:ddTraj.minst.PredictionHorizon]*ddTraj.Ts;
    tNow   = times(1);

    for ii=1:numIntervals
        tpts  = tNow + tSteps;
%         tpath = desTraj.xt.x(tpts);
%         tpath = tpath(1:2,:);
        
%         [mv, opt, info] = nlmpcmove(ddTraj.minst, xNow, uNow, [], [], options);
        [mv, opt, info] = nlmpcmove(ddTraj.minst, xNow, uNow, [], [], options);
        % @todo Figure out if need to return options to update.
        
        c = info.Cost;

        mPath.u = info.MVopt;
        mPath.y = info.Yopt;
        mPath.x = info.Xopt;
        mPath.t = info.Topt;

        dSol.x = [dSol.x , transpose(mPath.x(2:end,:))];
        dSol.u = [dSol.u , transpose(mPath.u(2:end,:))];
        dSol.y = [dSol.y , transpose(mPath.y(2:end,:))];
        dSol.t = [dSol.t , tNow + transpose(mPath.t(2:end,:))];

        tNow = dSol.t(end);
        xNow = dSol.x(:,end);
        uNow = dSol.u(:,end);
        
%         t = [t; tNow + mPath.t(end)];
%         w = [w; mPath.y(end, :)];
    end %! for
      %!DEBUG
      %!cnt

      %!--[3] Iterate over remainder.
      %!
    %  disp('dddddddddddddddddd');
    %  tNow
    %  if (tNow < desTraj.tspan(2)) 
    %    tpts  = tNow + tSteps;
    %    tpts - desTraj.tspan(2);
    %    iStop = find( (tpts - desTraj.tspan(2)) > 0 , 1, 'first') + 1;
    %    if (~isempty(iStop))
    %      tpts(iStop:end) = [];
    %    end
    %    tpts
    %    if (isempty(tpts))
    %      tpts = desTraj.tspan(2)
    %    elseif (tpts(end) < desTraj.tspan(2))
    %      tpts(end+1) = desTraj.tspan(2);
    %    end
    %    tpts
    %
    %    tpath = desTraj.x(tpts)
    %  
    %    [mPath, opt, info] = applyOneMove(this, xNow, uNow, tpath, options);
    %
    %    dSol.x = [dSol.x , transpose(mPath.x(2:end,:))];
    %    dSol.u = [dSol.u , transpose(mPath.u(2:end,:))];
    %    dSol.y = [dSol.y , transpose(mPath.y(2:end,:))];
    %    dSol.t = [dSol.t , tNow + transpose(mPath.t(2:end,:))];
    %    mPath.t
    %
    %    tNow = dSol.t(end);
    %  end
    %  tNow

    %!--[3] Repackage
    %!
%     rpSol = ddTraj.repackage(dSol);
    xIn   = {dSol.t , [1:6]};
    tspan = [min(dSol.t),max(dSol.t)];

    dSol.u(:,end+1) = dSol.u(end);
    x_v   = [dSol.x; [1, 0; 0 0; 0 1]*dSol.u];

    ddCurve = curve.interpolate(dSol.t, x_v);
    rpSol   = trajectory.path(ddCurve, tspan);
    
    w = rpSol.xt.x(times);
    t = times;
    w = w';
    %!DEBUG
    %   figure(100);
    %   tpath = desTraj.xt.xt.x(dSol.t);
    %   plot(dSol.x(1,:),dSol.x(2,:),'r-', ...
    %        tpath(1,:), tpath(2,:), 'g-.');
end
