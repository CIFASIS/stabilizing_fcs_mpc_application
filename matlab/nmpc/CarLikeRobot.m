function dxdt = CarLikeRobot(x, u, params)
    % Continuous-time state equations for the car like robot
    %
    % The states are:
    %   x(1) = px       Cartesian coordinate x [m]
    %   x(2) = py       Cartesian coordinate y [m]
    %   x(3) = theta    Orientation \theta [rad]
    %   x(4) = phi      Steering system angle [rad]
    %
    % The inputs are:
    %   u(1) = v        Linear speed [m/s]
    %   u(2) = omega    Steering system angular speed [rad/s]

    % states
    theta = x(3);
    phi = x(4);

    % inputs
    v = u(1);
    omega = u(2);

    % parameters
    L = params(1);

    % state equations
    dxdt = zeros(4,1);
    dxdt(1) = v * cos(theta);
    dxdt(2) = v * sin(theta);
    dxdt(3) = v / L * tan(phi);
    dxdt(4) = omega;
