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
function [A, B, c, C, D, e] = linearize(arm, ref, space)

% define parameters
x_est = arm.x.val;
uLast = arm.u.val;
eps = 1e-3;

% allocate memory for matrices
nStates = length(arm.x.val);
nInputs = length(arm.u.val);
nOutputs = length(ref);
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

% compute measurement matrix, C, and constant vector, d
switch space
    
    % output joint-space state
    case 'joint'
        C = eye(nOutputs,nStates);
        D = zeros(nOutputs,nInputs);
        e = zeros(nOutputs,1);
    
    % output task-space coordinates by linearizing forward kinematics
    case 'task'
        C = zeros(nOutputs,nStates);
        [g,~,~] = fwdKin(arm, x_est);
        for i = 1:nStates
            x_eps = x_est;
            x_eps(i) = x_eps(i) + eps;        % one state perturbed
            [g_eps,~,~] = fwdKin(arm, x_eps); % state-perturbed forward kinematics
            C(:,i) = (g_eps-g)/eps;
        end
        D = zeros(nOutputs,nInputs);
        e = g - C*x_est;
    
    % torque (generalized force) control
    case 'force'
        
        %%%%%%%%%
        % TO DO %
        %%%%%%%%%

    % joint space by default
    otherwise
        C = eye(nOutputs,nStates);
        D = zeros(nOutputs,nInputs);
        e = zeros(nOutputs,1);
end

end