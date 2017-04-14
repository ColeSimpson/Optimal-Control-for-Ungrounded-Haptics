clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked');

% Add MPC toolbox to the working directory -- seems to be problematic in
% windows 
% http://people.ee.ethz.ch/~mpt/3/
addpath( genpath([ fileparts(pwd) '/tbxmanager' ]));

%% Whatever initialization nonsense
params.Ts = 0.02;              % sampling time for discretization
params.ref = [ 0; 0; 25; 0; 0 ];   % where you want your system to go

% x = [ phi phi_dot theta theta_dot wc ]'
params.x.current = [10; 0; 1; 0; 300];  % current state of the system
params.x.min = -inf*ones(5,1);  % state constraitns
params.x.max =  inf*ones(5,1);

params.u.current = [0;0];
params.u.min = -inf*ones(2,1);   % control constraints
params.u.max =  inf*ones(2,1);

params.Q = diag([ 1, 1, 100, 1, 1 ]);
params.R = diag(1e-6*ones(2,1));
params.H = 5;

%% Simulate
x = params.x.current;     % history of states
u = params.u.current;     % history of commands

% loop for simulating
for i = 1:300

    % Compute MPC control
    u_opt = control( @(x,u) dxdt(x,u), params );

    % Update state
    [t,x_integrated] = ode45(@(t,x) dxdt(x,u_opt), [0 params.Ts], params.x.current);

    % Save variables from this round
    params.x.current = x_integrated(end,:)';
    params.u.current = u_opt;
    x = [x, params.x.current];
    u = [u, u_opt];

end


%% Plot results
time = 0:params.Ts:i*params.Ts;

figure
subplot( 2, 1, 1 )
    plot(time, x')
    ylabel 'State trajectories'
subplot( 2, 1, 2 )
    plot( time, u' )
    ylabel 'Control trajectories'
    xlabel 'Time (seconds)'

figure
    plot( x(1,:), x(3,:) )
    ylabel 'State 2'
    xlabel 'State 1'
