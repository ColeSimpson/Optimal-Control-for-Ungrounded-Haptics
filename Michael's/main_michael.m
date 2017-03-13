clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked');

% Add MPC toolbox to the working directory -- seems to be problematic in
% windows 
% http://people.ee.ethz.ch/~mpt/3/
addpath( genpath([ fileparts(pwd) '/tbxmanager' ]));

%% Whatever initialization nonsense
params.Ts = 0.02;              % sampling time for discretization
params.ref = [ 0; 1; 0; 0 ];   % where you want your system to go

params.x.current = [0;0;0;0];  % current state of the system
params.x.min = -10*ones(4,1);  % state constraitns
params.x.max =  10*ones(4,1);

params.u.current = [0;0];
params.u.min = -50*ones(2,1);   % control constraints
params.u.max =  50*ones(2,1);

params.Q = diag([ 1, 1, 0.01, 0.01 ]);
params.R = diag(1e-6*ones(2,1));
params.H = 30;

%% Simulate
x = params.x.current;     % history of states
u = params.u.current;            % history of commands

% loop for simulating
for i = 1:30

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
    plot( x(1,:), x(2,:) )
    ylabel 'State 2'
    xlabel 'State 1'
