clc; clear; close all;

%% Symbolic Derivation
syms x x_dot theta theta_dot x_b x_b_dot tau real
syms M_wheel M_body m_ball h d R r I_body I_wheel I_ball g real
syms x_ddot theta_ddot x_b_ddot real

%% KINETIC ENERGY (T_total)
% Wheel KE (translational + rotational with no-slip condition)
T_wheel = 0.5 * M_wheel * x_dot^2 + 0.5 * I_wheel * (x_dot/R)^2; 

% Body KE
% NOTE: I am assuming COM at height h/2
x_body = x + h/2 * sin(theta);
y_body = h/2 * cos(theta);
x_body_dot = x_dot + h/2 * cos(theta) * theta_dot;
y_body_dot = - h/2 * sin(theta) * theta_dot;
T_body_trans = 0.5 * M_body * (x_body_dot^2 + y_body_dot^2);
T_body_rot = 0.5 * I_body * theta_dot^2;
T_body = T_body_trans + T_body_rot;

% Ball KE
x_ball = x + (h + r) * sin(theta) + x_b * cos(theta);
y_ball = (h + r) * cos(theta) - x_b * sin(theta);

x_ball_dot = x_dot + (h + r) * cos(theta) * theta_dot + x_b_dot * cos(theta) - x_b * sin(theta) * theta_dot;
y_ball_dot = -(h + r) * sin(theta) * theta_dot - x_b_dot * sin(theta) + x_b * cos(theta) * theta_dot;

T_ball_trans = 0.5 * m_ball * (x_ball_dot^2 + y_ball_dot^2);
T_ball_rot = 0.5 * I_ball * (x_b_dot / r)^2;
T_ball = T_ball_trans + T_ball_rot;

% Total Kinetic Energy
T_total = T_wheel + T_body + T_ball;

%% POTENTIAL ENERGY (V_total)
% V_body and V_ball use the y_body and y_ball definitions from above
V_body = M_body * g * y_body;
V_ball = m_ball * g * y_ball;
V_total = V_body + V_ball;

%% LAGRANGIAN
L = T_total - V_total;

%% EULER-LAGRANGE EQUATIONS
% Generalized coordinates and velocities
q = [x; theta; x_b];
q_dot = [x_dot; theta_dot; x_b_dot];
q_ddot = [x_ddot; theta_ddot; x_b_ddot];

% Initialize equations
eqns = sym(zeros(3,1));

for i = 1:3
    % dL/dq_dot
    dL_dq_dot = diff(L, q_dot(i));
    
    % Time derivative of dL/dq_dot
    d_dt_dL_dq_dot = 0;
    for j = 1:3
        d_dt_dL_dq_dot = d_dt_dL_dq_dot + diff(dL_dq_dot, q(j)) * q_dot(j); %chain rule
    end
    for j = 1:3
        d_dt_dL_dq_dot = d_dt_dL_dq_dot + diff(dL_dq_dot, q_dot(j)) * q_ddot(j);
    end
    
    % dL/dq
    dL_dq = diff(L, q(i));
    
    % Generalized forces (motor torque)
    if i == 1
        Q = tau/R;
    elseif i == 2
        Q = -tau;
    else
        Q = 0;
    end
    
    % Euler-Lagrange equation
    eqns(i) = d_dt_dL_dq_dot - dL_dq - Q;
end

%% SOLVE FOR ACCELERATIONS
% Solve the system for accelerations [x_ddot, theta_ddot, x_b_ddot]
sol = solve(eqns, [x_ddot, theta_ddot, x_b_ddot]);

% Extract solutions
x_ddot_expr = sol.x_ddot;
theta_ddot_expr = sol.theta_ddot;
x_b_ddot_expr = sol.x_b_ddot;

%% DISPLAY THE EQUATIONS
 fprintf('\n--- Derived Equations of Motion ---\n\n');
 fprintf('x_ddot = %s\n', char(x_ddot_expr));
 fprintf('theta_ddot = %s\n', char(theta_ddot_expr));
 fprintf('x_b_ddot = %s\n', char(x_b_ddot_expr));

%% CONVERT TO NUMERICAL FUNCTIONS
% Convert symbolic expressions to MATLAB functions
vars_list = {x, x_dot, theta, theta_dot, x_b, x_b_dot, tau, M_wheel, M_body, m_ball, h, d, R, r, I_body, I_wheel, I_ball, g};
         
x_ddot_func = matlabFunction(x_ddot_expr, 'Vars', vars_list);
theta_ddot_func = matlabFunction(theta_ddot_expr, 'Vars', vars_list);
x_b_ddot_func = matlabFunction(x_b_ddot_expr, 'Vars', vars_list);

%% NUMERICAL PARAMETERS
params.M_wheel = 4.7; %Mw (mass of both wheels)
params.M_body = 9.13; %Mr
params.m_ball = 2.78; %mb
params.h = 0.3641; %h
params.d = 0.2201; %d
params.R = 0.361; %R
params.r = 0.04005; %r
params.I_body = 1/12 * params.M_body * (params.d^2 + params.h^2); % + params.M_body * params.d^2 ??
params.I_ball = 2 / 5 * params.m_ball * params.r^2;
params.I_wheel = params.M_wheel * params.R^2 / 2;
params.g = 9.81;

%% LINEARIZATION

% Symbolic state and input vectors
z_sym = [x; x_dot; theta; theta_dot; x_b; x_b_dot];
u_sym = [tau];

% Symbolic state-space function f(z, u)
% xd = Ax + Bu, where A = d(xd)/dx, B = d(xd)/du
f_sym = [x_dot;
         x_ddot_expr;
         theta_dot;
         theta_ddot_expr;
         x_b_dot;
         x_b_ddot_expr];

% Calculating Jacobians
A_sym = jacobian(f_sym, z_sym);
B_sym = jacobian(f_sym, u_sym);

% Defining the equilibrium point
% z = [x, xd, theta, thetad, x_b, x_bd]
z_eq = [0; 0; 0; 0; 0; 0];
u_eq = 0; % no torque at equilibrium

% Substitute numerical values to get A and B matrices
% List of all symbolic variables to be substituted
p = params;
sym_vars = {M_wheel, M_body, m_ball, h, d, R, r, I_body, I_wheel, I_ball, g, x, x_dot, theta, theta_dot, x_b, x_b_dot, tau};

% List of corresponding numerical values
num_vals = {p.M_wheel, p.M_body, p.m_ball, p.h, p.d, p.R, p.r, p.I_body, p.I_wheel, p.I_ball, p.g, z_eq(1), z_eq(2),...
            z_eq(3), z_eq(4), z_eq(5), z_eq(6), u_eq};

% Substitute all values at once and convert to double
A = double(subs(A_sym, sym_vars, num_vals));
B = double(subs(B_sym, sym_vars, num_vals));

% Define C and D matrices
% Output 1: Body Angle (theta)
% Output 2: Wheel Position (x)

% Assuming an encoder measures wheel orientation and converts it to linear
% displacement as x = R*phi
C = [0 0 1 0 0 0;   % y1 = theta
     1 0 0 0 0 0];  % y2 = x
D = [0;
    0];

% Create the Linear State-Space (LTI) object
sys = ss(A, B, C, D);

% To verify system controllability and observability
fprintf('\nRank of ctrb and obsv matrices:\n')
disp(rank(ctrb(sys)))
disp(rank(obsv(sys)))
% Both matrices are full rank and this confirms that the system is observable and controllable

fprintf('Linearization complete.\n');
fprintf('A matrix:\n');
disp(A);
fprintf('B matrix:\n');
disp(B);


%% SAVE RESULTS TO FILE
save('model_data.mat', ...
     'A', 'B', 'C', 'D', ...                             % Linear model
     'params', ...                                       % System parameters
     'z_eq', ...                                         % Equilibrium point
     'x_ddot_func', 'theta_ddot_func', 'x_b_ddot_func'); % Sim functions

fprintf('Saved results to model_data.mat\n');