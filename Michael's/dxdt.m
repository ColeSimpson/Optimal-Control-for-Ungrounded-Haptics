function xdot = dxdt( t, x, Tt, tt )
% function xdot = dxdt( t, x, Tt, tt )
% This function encodes the nonlinear dynamics of the system.  Replace this
% function with the dynamics of your system.  

if nargin == 4
    % Interpolate the data set (tt,Tt) at time t - allows a better
    % approximation than the zero-order hold
    T1 = interp1( tt, Tt(1,:), t, 'spline' ); 
    T2 = interp1( tt, Tt(2,:), t, 'spline' ); 
    T = [ T1; T2 ];
else
    T = zeros(2,1);
end

% hard code in parameters in this version
I1 = 1;
I2 = 1;
m1 = 1;
m2 = 1;
L1 = 1;
L2 = 1;
Lc1 = 0.5;
Lc2 = 0.5;
B = 0.2 * eye(2) + 0.05 * ones(2,2);

% Compute mass matrix
a1 = I1 + I2 + m2*L1^2;
a2 = m2 * L1 * Lc2;
a3 = I2;

M11 = a1 + 2*a2*cos(x(2));
M12 = a3 + a2*cos(x(2));
M21 = M12;
M22 = a3;
M = [M11 M12;
     M21 M22];
 
% Compute coriolis and centripetal effects
V1 = -x(4)*(2*x(3) + x(4));
V2 = x(3)^2;
V = [V1;V2] * a2*sin(x(2));

% % Compute Jacobian
% J = [(-L1*sind(x(1)) - L2*sind(x(1)+x(2))) -L2*sind(x(1)+x(2));
%      (L1*cosd(x(1)) + L2*cosd(x(1)+x(2)))   L2*cosd(x(1)+x(2))];

xdot = [ x(3:4); M\(T-V-(B)*x(3:4)) ];
