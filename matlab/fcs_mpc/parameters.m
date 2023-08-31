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
F_OmegaI = [-0.0162184150400002, -0.00631156082725452, -0.00162300350054704; 0.0162184150400002, 0.00631156082725452, 0.00162300350054704; -1.91252880199500e-16, 0.0704545731974788, 0; 1.91252880199500e-16, -0.0704545731974788, 0; 0.00203835199999989, 0.0823409037568373, 0.0338642741053293; -0.00203835199999989, -0.0823409037568373, -0.0338642741053293; -0.000739879228098683, -0.0271199969505346, 0.00314846365802843; 0.000739879228098683, 0.0271199969505346, -0.00314846365802843]; % F for \Omega_I
g_OmegaI = [0.00133150970594632; 0.00133150970594632; 0.00320850126341322; 0.00320850126341322; 0.00334621106108639; 0.00334621106108639; 0.00174333401990860; 0.00174333401990860]; % g for \Omega_I
F_OmegaO = [1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0; 0, 0, 1; 0, 0, -1]; % F for \Omega_O
g_OmegaO = [0.1; 0.1; 0.046; 0.046; 0.2; 0.2]; % g for \Omega_O
Hp = 23; % Prediction horizon (up to 25, bigger than that would require to recompute Lmin and Lmax)
Hc = 9; % Control horizon
Lmin = 0.0000953972409494; % \tilde{L}_{min}
Lmax = 17.5653329210839395; % \tilde{L}_{max}
r = 0.1; % Should be in [0, 1)
c = 2.1e-8; % 0; % Should satisfy c <= r * Lmin / (Hp * Lmax). Taking worst case: r * Lmin / (25 * Lmax) = 2.172398129383434e-08.
varying_cost = false; % Original implementation had time varying cost penalizing orientation error that depended on recalculated speed. In stabilizing FCS-MPC with inner-outer sets, this cost doesn't vary.
speed_cost_coefficient = 5 / 3.6; % Coefficient (partial) to penalize orientation error, used when varying_cost is false. It represents 2 km/h.

% Select trajectory and load noise
addpath('../data/');
load('trajectory_references.mat')
trajectory = trajectory_straight;
load('noise_02.mat')

% Initial conditions
initial_x = 0;
initial_y = 0.05;
initial_theta = deg2rad(9);
