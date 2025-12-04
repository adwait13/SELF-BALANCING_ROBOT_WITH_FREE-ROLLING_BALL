% Make sure to run derieve_model.m before you run this
% See line 40 to change initial conditions

clc; clear; close all;

% Load all the pre-computed data
fprintf('Loading model data...\n');
load('model_data.mat');

%% LQR CONTROLLER DESIGN
fprintf('Designing LQR controller...\n');

% Define Q and R matrices
Q = diag([1,    ... x
          1,    ... x_dot
          30,    ... theta
          80,    ... theta_dot
          1500,   ... x_b 
          100]);  ... x_b_dot

% R (Input Cost) -> cheap
R = 1;

% LQR Gain Matrix 'K'
K = lqr(A, B, Q, R);
fprintf('LQR Gain K calculated:\n');
disp(K);

%% Save Results
save('lqr_data.mat', ...
     'K', "Q", "R");
fprintf('Saved results to lqr_data.mat\n');

%% CLOSED-LOOP LQR SIMULATION

fprintf('Running LQR simulation.\n');
t_sim = 0:0.01:10;
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

% Initial condition (start few degrees from upright)
z0 = [0;            ... x
    0;              ... xd
    deg2rad(15);    ... theta
    0;              ... thetad
    -0.04;              ... xb
    0];             ... xbd
% z_eq is [0; 0; 0; 0; 0; 0]


robot_dynamics_ODE = @(t, z) ...
    robot_dynamics(t, z, ...
        -K * (z - z_eq), ...
        params, ...
        x_ddot_func, theta_ddot_func, x_b_ddot_func);

% Run the closed-loop simulation
[t_lqr, z_lqr] = ode45(robot_dynamics_ODE, t_sim, z0, options);
%[t_lqr, z_lqr] = ode45(@(t, z) (A * z + B * (-K * z)));

fprintf('Simulation complete.\n');

%% 11. PLOT LQR RESULTS
figure;
sgtitle('Closed-Loop LQR Control');

% Plot Body Angle
subplot(3,1,1);
plot(t_lqr, rad2deg(z_lqr(:,3)), 'b-', 'LineWidth', 2);
title('Body Angle');
ylabel('θ (deg)');
ylim([-20, 20]); % Zoom in 
grid on;

% Plot Wheel Position
subplot(3,1,2);
plot(t_lqr, z_lqr(:,1), 'b-', 'LineWidth', 2);
title('Wheel Position');
ylabel('x (m)');
grid on;

% Plot Ball Position
subplot(3,1,3);
plot(t_lqr, z_lqr(:,5), 'b-', 'LineWidth', 2);
title('Ball Position');
ylabel('x_b (m)');
xlabel('Time (s)');
grid on;
ylim([-0.12, 0.12]);


%%  Simulation of Linear Uncompensated System

t_sim = 0:0.01:10;
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

% --- Linear Simulation (Simulating the DEVIATION) ---
% The linear system simulates the deviation from equilibrium
% z_dev = z_abs - z_eq
% z0_dev = z0_lin - z_eq
z0_lin = [0; 0; deg2rad(0.01); 0; 0; 0];
tau_val = 0;
z0_dev = z0_lin - z_eq; 

% Anonymous function for linear dynamics (tau = 0)
linearODE = @(t, z_dev) A * z_dev + B * 1; 
[t_lin, z_dev] = ode45(linearODE, t_sim, z0_dev, options);

% Convert linear deviation back to absolute values for plotting
z_lin_abs = z_dev + z_eq'; % Add equilibrium back

%% PLOT FOR LINEARIZED SYSTEM
% Plot to validate the linearization
figure;
sgtitle('Linearization Validation');

% Plot Body Angle
subplot(3,1,1);
plot(t_lin, rad2deg(z_lin_abs(:,3)), 'r', 'LineWidth', 1.5);
title('Body Angle');
ylabel('θ (deg)');
grid on;

% Plot Wheel Position
subplot(3,1,2);
plot(t_lin, z_lin_abs(:,1), 'r', 'LineWidth', 1.5);
title('Wheel Position');
ylabel('x (m)');
grid on;

% Plot Ball Position
subplot(3,1,3);
plot(t_lin, z_lin_abs(:,5), 'r', 'LineWidth', 1.5);
title('Ball Position');
ylabel('x_b (m)');
xlabel('Time (s)');
grid on;

%{
%% ANIMATION OF LQR CONTROLLER
fprintf('Running animation...\n');

% Get sim results
t = t_lqr;
z = z_lqr;
p = params;

% Create a new figure for the animation
figure;
set(gcf, 'Position', [100, 100, 800, 600]); % Set window size
ax = gca;
ax.NextPlot = 'replaceChildren';

% Define animation frame rate
skip = 5; % Plot every 5th frame
pause_time = (t(2) - t(1)) * skip;

% Set up plot limits
plot_width = 2.0; % Width of the plot window in meters
plot_height = 2.0; % Height
y_base = -p.R - 0.2; % Y-level for the ground

% Loop through the simulation time
for i = 1:skip:length(t)
    % Get current state
    x_t = z(i, 1);
    theta_t = z(i, 3);
    x_b_t = z(i, 5);
    
    % Clear the axes
    cla(ax); 
    
    %Calculate component positions
    
    % Wheel
    wheel_center = [x_t, 0];
    
    % Body
    body_width = p.d;
    body_base = wheel_center;
    body_top_center = [x_t + p.h * sin(theta_t), p.h * cos(theta_t)];
    
    % Body corners
    perp_vec = [-cos(theta_t), sin(theta_t)]; % Perpendicular vector to body
    c1 = body_base + (body_width/2) * perp_vec;
    c2 = body_base - (body_width/2) * perp_vec;
    c3 = body_top_center - (body_width/2) * perp_vec;
    c4 = body_top_center + (body_width/2) * perp_vec;
    body_verts_x = [c1(1), c2(1), c3(1), c4(1)];
    body_verts_y = [c1(2), c2(2), c3(2), c4(2)];
    
    % Ball (using your exact derivation equations)
    ball_cx = x_t + (p.h + p.r) * sin(theta_t) + x_b_t * cos(theta_t);
    ball_cy = (p.h + p.r) * cos(theta_t) - x_b_t * sin(theta_t);
    ball_center = [ball_cx, ball_cy];
    
    % Draw components 
    hold(ax, 'on');
    
    % Draw ground
    plot(ax, [x_t - plot_width/2, x_t + plot_width/2], [-p.R, -p.R], 'w-', 'LineWidth', 2);
    
    % Draw body
    patch(ax, body_verts_x, body_verts_y, [0.8, 0.8, 0.8]);
    
    % Draw wheel
    viscircles(ax, wheel_center, p.R, 'Color', 'b', 'LineWidth', 2);
    
    % Draw wheel spoke (to show rotation)
    wheel_angle = -x_t / p.R; % Angle of wheel rotation
    spoke_end = [wheel_center(1) + p.R * cos(wheel_angle), wheel_center(2) + p.R * sin(wheel_angle)];
    plot(ax, [wheel_center(1), spoke_end(1)], [wheel_center(2), spoke_end(2)], 'b-');
    
    % Draw ball
    viscircles(ax, ball_center, p.r, 'Color', 'r', 'LineWidth', 2);
    
    hold(ax, 'off');
    
    % Format plot
    axis equal;
    xlim(ax, [x_t - plot_width/2, x_t + plot_width/2]);
    ylim(ax, [y_base, y_base + plot_height]);
    title(ax, sprintf('LQR Control - Time: %.2f s', t(i)));
    xlabel(ax, 'Position x (m)');
    ylabel(ax, 'Position y (m)');
    %grid on;
    
    % Pause for animation
    drawnow;
    % pause(pause_time);
end

fprintf('Animation complete.\n');
%}