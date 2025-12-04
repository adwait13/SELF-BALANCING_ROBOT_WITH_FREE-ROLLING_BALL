clc; clear; close all;
load('model_data.mat');
load('lqr_data.mat');

% Define Shuffling Logic (Map Old -> New)
idx_measured = [3, 1];          % theta, x
idx_remaining = [2, 4, 5, 6];   % velocities and ball

% Create Permutation Matrix P
P = eye(6);
T = P([idx_measured, idx_remaining], :); % The Shuffler

% Create Shuffled System
% New Order: [theta, x, xd, thetad, xb, xbd]
A_new = T * A * T'
B_new = T * B
C_new = C * T'
D_new = D
K_new = K * T'

fprintf('System reordered. New state vector is: [x; theta; x_dot; theta_dot; x_b; x_b_dot]\n');

%% Reduced Order Observer Design

% Since A_new is ALREADY shuffled, the measured states are at the top.
n_m = 2; % Number of measured states
n_u = 4; % Number of unmeasured states

idx_a = 1:n_m;          % Indices 1, 2
idx_b = n_m+1 : n_m+n_u; % Indices 3, 4, 5, 6

% Partition the new matrices
Aaa = A_new(idx_a, idx_a)
Aab = A_new(idx_a, idx_b)
Aba = A_new(idx_b, idx_a)
Abb = A_new(idx_b, idx_b)

Ba = B_new(idx_a)
Bb = B_new(idx_b)

% Design Gain Ke
p_o = (4 * eig(A_new-B_new*K_new)); % observer poles are 4 times faster
p_o = real(p_o(3:6, 1))
Ke = place(Abb', Aab', p_o)';

fprintf('Reduced Order Observer Gain Ke:\n');
disp(Ke);

Fhat = Bb - Ke * Ba 
Bhat = (Abb - Ke*Aab)*Ke + Aba - Ke*Aaa
Ahat = Abb - Ke * Aab
Chat = [0 0 0 0;
        0 0 0 0;
          eye(4)]
Dhat = [eye(2); ...
          Ke]

%% INITIAL CONDITION CALCULATION

% Initial conditions
% [theta; x; xd; thetad; xb; xbd]
x0_plant = [deg2rad(10); 0; 0; 0; 0; 0]; 

% Observer's initial guess
% We assume we trust our sensors (y), but we guess the unmeasured parts are 0.
% [theta, x, xd, thetad, xb, xbd]

% Extract the measured part (y) from the plant condition
y_start_guess = [x0_plant(1); x0_plant(2)];

% Guess the unmeasured parts (xu) are zero (velocities & ball)
xu_start_guess = [0; 0; 0; 0]; 

% 3. Calculate eta0 (The Math Translation)
% eta = x_unmeasured - Ke * y_measured
eta0_observer = double(xu_start_guess - Ke * y_start_guess);

fprintf('Calculated eta0 based on initial tilt:\n');
disp(eta0_observer);

%% Design for filter

% cutoff frequency fc
fc = 2 * pi * 15; % for 15Hz

disp("You can run the Reduced Order Observer now...")