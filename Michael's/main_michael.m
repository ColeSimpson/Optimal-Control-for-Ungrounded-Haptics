clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked');

% Add MPC toolbox to the working directory -- seems to be problematic in windows
% http://people.ee.ethz.ch/~mpt/3/
addpath( genpath([ fileparts(pwd) '/tbxmanager' ]));

%% Whatever initialization nonsense


%% Simulate
x_current = [];  % current state of the system

x = x_current;     % history of states
u = [];            % history of commands


% loop
for i = 1:10

    % Compute MPC control
    u_opt = control( @(u) dxdt(x_current,u), ref );

    % Update state
    [t,x_integrated] = ode45(@(t,x) dxdt(x,u_opt), [0 Ts], x_current);

    % Save variables from this round
    x_current = x_integrated(:,end);
    x = [x, x_current];
    u = [u, u_opt];

end


%% Plot results
time = 1:Ts:i/Ts;

subplot( 2, 1, 1 )
    plot(time, x')
    ylabel 'State trajectories'
subplot( 2, 1, 2 )
    plot( time, u' )
    ylabel 'Control trajectories'
    xlabel 'Time (seconds)'
