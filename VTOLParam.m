% Ballbeam Parameter File

% Physical parameters of the ballbeam known to the controller
P.ml = 0.25;    % kg
P.mr = 0.25;    % kg
P.mc = 1;       % kg
P.d = 0.5;      % m
P.g = 9.81;     % m/s^2
P.Jc = .0042;   % kg m^2 inertia constant
P.u = 0.1;      % kg/s u constant

% parameters for animation
% set here


% Initial Conditions
P.z0 = 0.0;                % initial VTOL horizontal position, m
P.h0 = 0.0;                % initial VTOL height, m
P.theta0 = 0.0;            % initial VTOL angle, rads
P.zdot0 = 0.0;             % initial VTOL horizontal velocity, m/s
P.hdot0 = 0.0;             % initial VTOL height velocity, m/s
P.thetadot0 = 0.0;         % initial beam angular velocity, rads/s

% Simulation parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 99.9;   % End time of simulation
P.Ts = 0.01;      % sample time for controller
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% dirty derivative parameters
% set here
P.sigma = .05;
P.beta = (2*P.sigma - P.Ts)/(2*P.sigma + P.Ts);

% uncertainty
P.uncertainty = 1;
P.alpha = .2;
P.randomUncertainty = (1 + 2*P.alpha*rand - P.alpha);

