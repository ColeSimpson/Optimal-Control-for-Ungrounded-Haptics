function u = control( nonlinmodel, params )
% This function computes the MPC control output for a model of the arm
% tracking a reference state in joint, task (Cartesian), or force space. It
% employs the Multi-Parametric Toolbox MPT3. The MPT model is created by
% first linearizing the arm model (both dynamics and output) about its
% current state estimate.
%
% ________________________________
% |                              |
% | Control                      |
% |   linearize  dx/dt = f(x,u)  |
% |   discretize dx/dt = Ax+Bu+c |              _________
% |   linearize  y = g(x)        |       u*     |       |   x
% |                              |______________| Plant |____
% |   min  y'Qy + u'Ru           |       |      |_______|
% |   s.t. dx/dt = Ax+Bu+c       |       |          |
% |        y = Cx+d              |       |          |
% |        x_min <= x <= x_max   |       |          |
% |        u_min <= u <= u_max   |       |          | x_sens
% |______________________________|       |          |
%                |                _______|_____     |
%                |                |           |     |
%                |________________| Estimator |_____|
%                  x_est          |___________|
%
%
% The function also outputs a flag. 'flag = 0' implies no problems. 'flag
% = 1' signals that the linearization failed and the function returns
% without attempting to compute the optimal control. The 'params' input is
% optional.


% linearize arm model (and check linearization)
[ A, B, f ] = linearize( nonlinmodel, params.x.current, params.u.current );

% convert dynamics to discrete time -- may be a native function for this
[Ad, Bd, fd] = discretize( params.Ts, A, B, f );

% define LTI model for MPT3 package
model = LTISystem('A',Ad,'B',Bd,'f',fd,'Ts', params.Ts);
% model = LTISystem('A',Ad,'B',Bd,'f',fd,'C',C,'D',D,'g',g,'Ts',params.Ts);
% system can be affine with the form:
%   x_dot = Ax + Bu + f
%       y = Cx + Du + g

% make model track a reference
model.x.with('reference');
model.x.reference = 'free';

% define constraints
model.x.min = params.x.min;   % state constraints
model.x.max = params.x.max;
model.u.min = params.u.min;   % control constraints
model.u.max = params.u.max;

% define cost function (NOTE: this assumes that half of the outputs are
% positions and half are velocities)
model.x.penalty = QuadFunction( params.Q );
model.u.penalty = QuadFunction( params.R );

% create MPC controller
mpc = MPCController(model, params.H);
% u = mpc.evaluate( params.x.current );
u = mpc.evaluate( params.x.current, 'x.reference', params.ref);


end







function [A, B, c ] = linearize( nonlinmodel, state, ctrl )
% This function linearizes the dynamics and output equation for an arm
% model. It computes the 1st-order Taylor series approximation of
% dynamics dx/dt = f(x,u) and output (e.g., forward kinematics) y = g(x,u)
% about the current state estimate and most recent control. The dynamics
% will take the (affine) form dx/dt = f(x,u) = Ax + Bu + c, where A = df/dx
% and B = df/du. The output equation will take the (affine) form y = g(x) =
% Cx + Du + e, where C = dg/dx and D = dg/du. Both linear approximations
% are only reasonable for x "close" to x_est, which we assume to hold over
% MPC's finite horizon. For example, the linearization of the dynamics
% proceeds as follows:
%
%   f(x,u) = f(x_k,u_k) + df/dx*(x - x_k) + df/du*(u - u_k) + H.O.T.
%   f(x,u) ~ (df/dx)*x + (df/du)*u + [f(x_k,u_k) - (df/dx)*x_k - (df/du)*u)k]
%   f(x,u) ~ Ax + Bu + c
%
% As noted above, A = df/dx and B = df/du in this final equation. Assuming
% small perturbations (dx and du), derivatives are approximated via
% Euler's method:
%
%   df/dx @ x_k ~ [f(x_k+dx)-f(x_k)]/dx
%   df/du @ u_k ~ [f(u_k+du)-f(u_k)]/du
%
% The math is similar for the linearization of the output equation.


% define parameters
eps = 1e-3;

% allocate memory for matrices
nStates = length(state);
nInputs = length(ctrl);
A = zeros(nStates);
B = zeros(nStates,nInputs);

% compute dynamics matrix, A
f = nonlinmodel( state, ctrl );
for i = 1:nStates
x_eps = state;
x_eps(i) = x_eps(i) + eps;    % one state perturbed
f_eps = nonlinmodel(x_eps, ctrl); % state-perturbed dynamics
A(:,i) = (f_eps-f)/eps;
end

% compute input-to-state matrix, B
for i = 1:nInputs
u_eps = ctrl;
u_eps(i) = u_eps(i) + eps;           % one input perturbed
f_eps = nonlinmodel(state, u_eps); % input-perturbed dynamics
B(:,i) = (f_eps-f)/eps;
end

% compute constant vector, c
c = f - A*state - B*ctrl;


end
