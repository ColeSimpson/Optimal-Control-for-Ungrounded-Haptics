clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked');

% Add MPC toolbox to the working directory
% http://people.ee.ethz.ch/~mpt/3/
addpath( genpath([ pwd '/tbxmanager' ]));

%% Heather's Transfer function
a = 2; % amplitude of vibrations
m = 0.00575; % mass of magnet (kg) 0.00725
b = 0.8; % damping constant of actuator (Ns/m)
k = 1159; % spring constant of actuator (N/m)
mh = 0.029; % effective mass of hand (kg)
bu = 8; % damping constant of hand (Ns/m)
ku = 509; % spring constant of hand (N/m)
Bl = 0.79; % actuator parameter

fr = 0.05;     % limit of stiction

% transfer function from actuator current to force output
numF = Bl*[2*m 0 0];
denF = [m 2*b 2*k];
sysF = tf(numF,denF);
sysd = c2d(sysF, 0.0001);   % discretize

[ A, B, C, D ] = tf2ss( sysd.num{1}, sysd.den{1} ); % convert to state-space

%% MPC simulation

%%% Define MPC model as a piecewise function that includes a model of
%%% friction
% Stiction portion
sys1 = LTISystem('A', A, 'B', B, 'C', [0, 0], 'D', 0 );
R_1 = Polyhedron( 'lb', 0*ones(3,1), 'ub', fr*ones(3,1) );

% Dynamic friction
sys2 = LTISystem('A', A, 'B', B, 'C', C, 'D', D );
R_2 = Polyhedron( 'lb', fr*ones(3,1) );

sys1.setDomain('xu', R_1);
sys2.setDomain('xu', R_2);

model = PWASystem([sys1, sys2]);


% Define limits - can put limits on x, y, or u
model.u.min = -0.5;
model.u.max = 0.5;

% Define penalties on state and actuation for optimization (same idea as
% LQR)
% model.x.penalty = QuadFunction( eye(2));
model.y.penalty = QuadFunction( 1 );
model.u.penalty = QuadFunction( 0.00001 );

% add reference
model.y.with('reference');
model.y.reference = 'free';

% Define the MPC controller 
H = 20; % number of time points to look into the future for optimization
ctrl = MPCController(model, H); 

x0 = [0; 0];  % initial state

% Simulate the response of the system with the MPC controller
loop = ClosedLoop(ctrl, model);  % simulates closed-loop response
Nsim = 100;  % number of steps to simulate
data = loop.simulate(x0, Nsim, 'y.reference', -10);  % simulate!

%% Plot results
subplot(3,1,1)
plot( data.X' )
hold on
ylabel 'magnet position'

subplot(3,1,2)
plot(data.Y');
hold on;
% plot(1:Nsim, ys*ones(1, Nsim), 'k--')
ylabel('force (N)')

subplot(3,1,3)
plot(data.U');
hold on;
% plot(1:Nsim, us*ones(1, Nsim), 'k--')
ylabel('current (amps)')

