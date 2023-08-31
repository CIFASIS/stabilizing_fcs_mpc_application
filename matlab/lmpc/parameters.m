% System parameters
step_time = 0.2; % Sampling time
sampling_scale = 5; % Amount of states logged between sampling instants
Kp = 20; % Speed controller proportional gain
Ti = 1.2; % Speed controller integral time
K = 1; % Driver/converter gain
speed_allow_direction = 0.1407; % To avoid turning when robot is stationary
phi_max = 0.61; % Steering limit
Lr = 0.79; % Length from center to rear axis
Lf = 0.79; % length from center to front axis
L = Lr + Lf; % Total length
Lw = 0.54; % Length from center to left side
R = 0.29; % Wheel radius
M = 250; % Vehicle mass
J = 50; % Vehicle inertia (with respect to center of mass)
Jw = 5; % Wheel inertia
Bw = 1.7; % Wheel internal friction
Bf = 10000; % Friction coefficient wheel-floor
minimal_distance = 0.05; % Minimal distances between positions (used to calculate RMSE)

% Control parameters
initial_speed = 5 / 3.6; % 2 km/h
initial_dir = 0;
A = [0,                                initial_speed*tan(initial_dir)/L, 0,             0;...
    -initial_speed*tan(initial_dir)/L, 0,                                initial_speed, 0;...
    0,                                 0,                                0,             initial_speed/L/cos(initial_dir)^2;...
    0,                                 0,                                0,             0];
B = [1 0; 0 0; 0 0; 0 1];
C = eye(4);
D = zeros(4, 2);
linsys1 = ss(A, B, C, D);
linsys1 = c2d(linsys1, step_time);
linsys1.InputName = {'verror', 'omegaerror'};
linsys1.OutputName = {'xerror', 'yerror', 'thetaerror', 'phierror'};
linsys1.StateName = {'xerror', 'yerror', 'thetaerror', 'phierror'};
linsys1.InputGroup.MV = 2;
linsys1.OutputGroup.MO = 4;
Hp = 23; % Prediction horizon
Hc = 9; % Control horizon
mpc1 = mpc(linsys1);
mpc1.PredictionHorizon = Hp;
mpc1.ControlHorizon = Hc;
mpc1.Model.Nominal.U = [0; 0]; % Nominal values for inputs
mpc1.Model.Nominal.Y = [0; 0; 0; 0]; % Nominal values for outputs
mpc1.Weights.MV = [0 0]; % Input weights
mpc1.Weights.MVRate = [0 0]; % Input rate weights
mpc1.Weights.OV = [1, 1, initial_speed*step_time, (initial_speed*step_time)^2/L]; % Cost similar to FCS-MPC
mpc1.Weights.ECR = 100000;
options = mpcsimopt(); % Specify simulation options
options.RefLookAhead = 'on';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';

% Select trajectory and load noise
addpath('../data/');
load('trajectory_references.mat')
trajectory = trajectory_straight;
load('noise_02.mat')

% Initial conditions
initial_x = 0;
initial_y = 0.05;
initial_theta = deg2rad(9);
