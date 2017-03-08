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

function [u, flag] = control(armModel, ref, space, params)

% if necessary, set default parameter values
if nargin < 4
    params.H = floor(armModel.Tr/armModel.Ts) + 1; % MPC horizon (until can next reoptimize)
    params.wP = 1;                                 % position cost
    params.wV = 5e-3;                              % velocity cost
    params.wU = 1;                                 % control cost
    params.alpha = 1e10;                           % weighting between state (pos/vel) and control costs
end

% linearize arm model (and check linearization)
[A, B, f, C, D, g] = linearize(armModel, ref, space);
if ~isreal(A) || sum(sum(isnan(A))) > 0
    flag = 1;
    return
end

% discretize dynamics of arm model
[Ad, Bd, fd] = discretize(armModel.Ts, A, B, f);

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