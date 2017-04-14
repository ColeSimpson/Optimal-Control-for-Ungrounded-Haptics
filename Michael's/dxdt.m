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


%------------ add "dx/dt" from MG ------------%
theta_dot_dot = x(2)*((Ixx-Iyy-Izz)*x(5)*sin(x(1))/(Iyy+m*L^2)+...
    (Ixx-Iyy-Izz)*x(5)*cos(x(1))^2/((Iyy+m*L^2)*sin(x(1)))+...
    (Izz-Ixx-Iyy-2*m*L^2)*x(4)/((Iyy+m*L^2)*tan(x(1))));

phi_dot_dot = sin(x(1))*(g*L*m-(Iyy-Ixx-Izz)*x(5)*x(4)-...
    (Izz-Iyy-m*L^2)*cos(x(1))*x(4)^2)/(Ixx+m*L^2);

wc_dot = x(2)*((Ixx-Iyy-Izz)*x(5)/((Iyy+m*L^2)*tan(x(1)))+...
    2*(Izz-Iyy-m*L^2)^2*sin(x(1))*cos(x(1))^2*x(4)/(Izz*(Iyy+m*L^2))+...
    (Izz-(Izz-Iyy-m*L^2)*sin(x(1))^2)*(Izz-Ixx-Iyy-2*m*L^2-2*(Izz-Iyy-...
    m*L^2)*sin(x(1))^2)*x(4)/(Izz*(Iyy+m*L^2)*sin(x(1))));
%------------end "dx/dt" section ---------------%


xdot = [ x(2); phi_dot_dot; x(4); theta_dot_dot; wc_dot ];
