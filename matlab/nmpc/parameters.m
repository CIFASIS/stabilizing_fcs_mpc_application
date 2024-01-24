% System parameters
step_time = 0.2; % Sampling time
sampling_scale = 5; % Amount of states logged between sampling instants
Kp = 20; % Speed controller proportional gain
Ti = 1.2; % Speed controller integral time
K = 1; % Driver/converter gain
omega_f = 0.3253; % Steering system speed (fixed value)
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
nx = 4;
ny = 4;
nlobj = nlmpc(nx, ny, 'MV', [2], 'MD', [1]);
Hp = 23;
Hc = 9;
nlobj.Ts = step_time;
nlobj.PredictionHorizon = Hp;
nlobj.ControlHorizon = Hc;
params = {[L, step_time, initial_speed]};
nlobj.Model.NumberOfParameters = length(params);
nlobj.Model.StateFcn = "CarLikeRobot";
nlobj.States(1).Name  = 'x';
nlobj.States(2).Name  = 'y';
nlobj.States(3).Name  = '\theta';
nlobj.States(4).Name  = '\delta';
nlobj.Model.OutputFcn = @(x,u,params) x;
nlobj.OV(1).Name  = 'x';
nlobj.OV(2).Name  = 'y';
nlobj.OV(3).Name  = '\theta';
nlobj.OV(4).Name  = '\delta';
nlobj.OV(4).Min  = -phi_max;
nlobj.OV(4).Max  = phi_max;
nlobj.OV(4).MinECR  = 1000;
nlobj.OV(4).MaxECR  = 1000;
nlobj.MV(1).Name = '\omega';
nlobj.MV(1).Min = -omega_f;
nlobj.MV(1).Max = omega_f;
nlobj.MD(1).Name = 'v';
nlobj.Optimization.CustomCostFcn = "CustomCostFcn";
nlobj.Optimization.ReplaceStandardCost = true;
x0 = [0; 0; 0; 0];
u0 = [0; 0];
validateFcns(nlobj,x0,u0(2),u0(1),params);
nloptions = nlmpcmoveopt;
nloptions.Parameters = params;
mdl = 'nmpc';
open_system(mdl)
createParameterBus(nlobj,[mdl '/Nonlinear MPC Controller'],'myBusObject',{[L, step_time, initial_speed]});

% Select trajectory and load noise
addpath('../data/');
load('trajectory_references.mat')
trajectory = trajectory_straight;
load('noise_02.mat')

% Initial conditions
initial_x = 0;
initial_y = 0.05;
initial_theta = deg2rad(9);
