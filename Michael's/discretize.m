% This function discretizes a linear (or affine) dynamical system using a
% zero-order hold approximation. The derivation for an affine system is as
% follows:
%
%   dx/dt = Ax + Bu + c
%   (x+ - x)/Ts = Ax + Bu + c
%   x+ = (Ax + Bu + c)*Ts + x
%   x+ = (A*Ts+I)x + (B*Ts)u + (c*Ts)
%   x+ = Ad*x + Bd*u + cd
%
% The final input c is optional (i.e., not necessary a purely linear system
% dx/dt = Ax + Bu). 
function [Ad, Bd, cd] = discretize(Ts, A, B, c)

Ad = A*Ts + eye(size(A));
Bd = B*Ts;

% handle purely linear case
if nargin < 4
    cd = zeros(size(A,1),1);
else
    cd = c*Ts;
end

end