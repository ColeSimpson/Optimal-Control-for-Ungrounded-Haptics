clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked');

% Add MPC toolbox to the working directory
% http://people.ee.ethz.ch/~mpt/3/
addpath( genpath([ fileparts(pwd) '/tbxmanager' ]));

%% Whatever initialization nonsense

%% Simulate

% loop
for i = 1:10
    u_opt = control( @(u) dxdt(x_current,u), ref );
    dxdt( x_current, u_opt );
end
