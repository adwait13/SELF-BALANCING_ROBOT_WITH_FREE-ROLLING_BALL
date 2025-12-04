clc; close all; clear;

fprintf('Loading model data...\n');
load('model_data.mat');
load('lqr_data.mat');

p_o = 4 * eig(real(A-B*K))

Ke = place(A', C', p_o)';

% initial condtions

x0 = [0;            ... x
    0;              ... xd
    deg2rad(15);    ... theta
    0;              ... thetad
    0;              ... xb
    0];             ... xbd

%% observer calculations

x0_obsv = [0; 0; 0; 0; 0; 0]; % initial estimates
x_eq = z_eq;

fc = 2 * pi * 15;

disp("You can run the Full State Observer now...")