function [u, flag] = control( nonlinmodel, ref )
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
[A, B, f, C, D, g] = linearize( nonlinmodel );
if ~isreal(A) || sum(sum(isnan(A))) > 0
    flag = 1;
    return
end

% discretize dynamics of arm model
[Ad, Bd, fd] = discretize( armModel.Ts, A, B, f );

% define LTI model for MPT3 package
model = LTISystem('A',Ad,'B',Bd,'f',fd,'C',C,'D',D,'g',g,'Ts',armModel.Ts);

% make model track a reference
model.y.with('reference');
model.y.reference = 'free';

% set (hard) constraints
model.x.min = armModel.x.min;   model.x.max = armModel.x.max;
model.u.min = armModel.u.min;   model.u.max = armModel.u.max;

% define cost function (NOTE: this assumes that half of the outputs are
% positions and half are velocities)
nInputs = length(armModel.u.val);
nOutputs = length(ref);
model.u.penalty = QuadFunction( diag(params.wU*ones(nInputs,1)) );
model.y.penalty = QuadFunction(  params.alpha * ...
    diag([params.wP*ones(nOutputs/2,1) ; params.wV*ones(nOutputs/2,1)]) );

% create MPC controller
ctrl = MPCController(model, params.H);

% simulate closed-loop system to find optimal control
loop = ClosedLoop(ctrl, model);
Nsim = params.H - 1; % one step more than optimization horizon
data = loop.simulate(armModel.x.val, Nsim, 'y.reference', ref);
u = data.U;
flag = 0;

end







function [A, B, c, C, D, e] = linearize( nonlinmodel, state )
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
nOutputs = length(nonlinmodel(state));
A = zeros(nStates);
B = zeros(nStates,nInputs);

% compute dynamics matrix, A
f = dynamics(arm, x_est);
for i = 1:nStates
x_eps = x_est;
x_eps(i) = x_eps(i) + eps;    % one state perturbed
f_eps = dynamics(arm, x_eps); % state-perturbed dynamics
A(:,i) = (f_eps-f)/eps;
end

% compute input-to-state matrix, B
for i = 1:nInputs
u_eps = uLast;
u_eps(i) = u_eps(i) + eps;           % one input perturbed
f_eps = dynamics(arm, x_est, u_eps); % input-perturbed dynamics
B(:,i) = (f_eps-f)/eps;
end

% compute constant vector, c
c = f - A*x_est - B*uLast;


end
