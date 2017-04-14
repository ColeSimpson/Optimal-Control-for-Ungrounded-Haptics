function xdot = dxdt( x, t, Tt, tt )
% function xdot = dxdt( x, t, Tt, tt )
% This function encodes the nonlinear dynamics of the system.  Replace this
% function with the dynamics of your system.  

% ---- include constants from MotionGenisis computation ------%
   g = 9.81; %m/s^2         % Earth's local gravity.
   L = 0.0;  %m %0.2  m             % Distance between No and Ccm.
   ro = 0.17;  %m             % Rotor radius.
   ri = 0.15;  %m
   h = 0.02;  %m
   m = 0.1; %kg
   Ixx = 1/12*m*(3*(ro^2 + ri^2)+h^2);
   Iyy = 1/12*m*(3*(ro^2 + ri^2)+h^2);
   Izz = 1/2*m*(ro^2 + ri^2);
%---- end of section of MotionGenisis Constants --------%

if nargin == 4
    % Interpolate the data set (tt,Tt) at time t - allows a better
    % approximation than the zero-order hold
    T1 = interp1( tt, Tt(1,:), t, 'spline' ); 
    T2 = interp1( tt, Tt(2,:), t, 'spline' ); 
    T = [ T1; T2 ];
else
    T = t;
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
%------------ add "dx/dt" from MG ------------%
theta_dot_dot = x(2)*((Ixx-Iyy-Izz)*x(5)*sin(x(1))/(Iyy+m*L^2)+(Ixx-Iyy-Izz)*x(5)*cos(x(1))^2/((Iyy+m*L^2)*sin(x(1)))+(Izz-Ixx-Iyy-2*m*L^2)*x(4)/((Iyy+m*L^2)*tan(x(1))));

phi_dot_dot = sin(x(1))*(g*L*m-(Iyy-Ixx-Izz)*x(5)*x(4)-(Izz-Iyy-m*L^2)*cos(x(1))*x(4)^2)/(Ixx+m*L^2);

wc_dot = x(2)*((Ixx-Iyy-Izz)*x(5)/((Iyy+m*L^2)*tan(x(1)))+2*(Izz-Iyy-m*L^2)^2*sin(x(1))*cos(x(1))^2*x(4)/(Izz*(Iyy+m*L^2))+(Izz-(Izz-Iyy-m*L^2)*sin(x(1))^2)*(Izz-Ixx-Iyy-2*m*L^2-2*(Izz-Iyy-m*L^2)*sin(x(1))^2)*x(4)/(Izz*(Iyy+m*L^2)*sin(x(1))));
%------------end "dx/dt" section ---------------%
xdot = [ x(2); phi_dot_dot; x(4); theta_dot_dot; wc_dot ];
