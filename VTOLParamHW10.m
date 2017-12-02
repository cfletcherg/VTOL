% parameters are separated into two files -- one that are specific for the
% homework set (here) and the other that contains values that won't change.
% These can be combined into one, but be sure to track homework specific 
% parameters carefully.

% inverted ballbeam - parameter file for hw8
VTOLParam % general ballbeam parameters

% tuning parameters
% this could include your damping ration for inner/outer loop, desired rise
% times, separation value between inner loop and outer, et.

% saturation limits for beam angle and total force
Tr_theta = .2;
Tr_z = Tr_theta*10;
wn_theta = 2.2/Tr_theta;
wn_theta_obs = 22*wn_theta;
zeta = .9;
wn_z = 2.2/Tr_z;
wn_z_obs = 40*wn_z;
Tr_h = 1.8;
wn_h = 2.2/Tr_h;
wn_h_obs = 40*wn_h;

% PD design for inner loop
% calculate the kp and kd gains for theta here...
P.kp_th  = wn_theta^2*(2*P.d^2*P.ml+P.Jc); % kp - inner
P.kd_th  = 2*zeta*wn_theta*(2*P.d^2*P.ml+P.Jc); % kd - inner

% DC gain for inner loop
k_DC_th = 1;

% integrator pole
integrator_pole = -20;

% PD design for outer loop
% calculate the kp and kd gains for theta here...
P.kp_z   = (wn_z)^2/-P.g; % kp - outer
P.kd_z   = (2*zeta*(wn_z)-P.u/(P.mc+2*P.ml))/-P.g; % kd - outer
P.ki_z   = -.0005; % ki - outer

% PID design for height
P.kp_h = .1134;
P.kd_h = .5833;
P.ki_h = .05;

% Saturation limits for beam angle and total force
P.theta_max = 30;
P.Fmax = 20;
P.tauMax = 20;

fe = P.g*(P.mc+2*P.ml);

%---------------------
% state space design 
P.Alat = [0 0 1 0;...
          0 0 0 1;...
          0 -fe/(P.mc+2*P.ml) -P.u/(P.mc+2*P.ml) 0;...
          0 0 0 0];
P.Blat = [0;...
          0;...
          0;...
          1/(P.Jc+2*P.ml*P.d^2)];
P.Clat = [1 0 0 0;...
          0 1 0 0];
Clat_o = [1 0 0 0];

P.Alon = [0 1;...
          0 0];
P.Blon = [0;...
          1/P.mc + 2*P.mr];
P.Clon = [1 0];

A1lon = [P.Alon, zeros(2,1);... 
         -P.Clon, 0];
B1lon = [P.Blon;...
         0];
A1lat = [P.Alat, zeros(4,1);...
         -Clat_o, 0];
B1lat = [P.Blat;...
         0];

% gains for pole locations
des_char_poly_lat = conv(conv(...
    [1,2*zeta*wn_z,wn_z^2],...
    [1,2*zeta*wn_theta,wn_theta^2]),...
    poly(integrator_pole));
des_poles_lat = roots(des_char_poly_lat);

des_char_poly_lon = conv([1,2*zeta*wn_h,wn_h^2], poly(integrator_pole));
des_poles_lon = roots(des_char_poly_lon);

% is the system controllable?
rank(ctrb(P.Alat,P.Blat));
rank(ctrb(P.Alon,P.Blon));

K1lat = place(A1lat,B1lat,des_poles_lat);
K1lon = place(A1lon,B1lon,des_poles_lon);
P.Klat = K1lat(1:4);
P.kilat = K1lat(5);
P.Klon = K1lon(1:2);
P.kilon = K1lon(3);

% observer design - lat
des_obsv_char_poly_lat = conv(...
    [1,2*zeta*wn_z_obs,wn_z_obs^2],...
    [1,2*zeta*wn_theta_obs,wn_theta_obs^2]);
des_obsv_poles_lat = roots(des_obsv_char_poly_lat);

rank(obsv(P.Alat,P.Clat));
P.Llat = place(P.Alat',P.Clat',des_obsv_poles_lat)';

% observer design - lon
des_obsv_char_poly_lon = [1,2*zeta*wn_h_obs,wn_h_obs^2];
des_obsv_poles_lon = roots(des_obsv_char_poly_lon);

rank(obsv(P.Alon,P.Clon));
P.Llon = place(P.Alon',P.Clon',des_obsv_poles_lon)';



