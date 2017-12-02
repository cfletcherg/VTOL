clear all
VTOLParamHW10;  % load parameters

% instantiate ballbeam, controller, and reference input classes 
% Instantiate Dynamics class
VTOL = VTOLDynamics(P);  
ctrl = VTOLController(P);  

amplitude_z = 3; % amplitude of reference z input
frequency_z = 0.05; % frequency of reference z input
reference_z = signalGenerator(amplitude_z, frequency_z);
amplitude_h = 1.0; % amplitude of reference h input
frequency_h = 0.05; % frequency of reference h input
reference_h = signalGenerator(amplitude_h, frequency_h);

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = VTOLAnimation(P);
observerPlot = plotObserverData(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input_z = reference_z.square(t);
    ref_input_h = 2 + reference_h.square(t);
    ref_input = [ref_input_z; ref_input_h];
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, VTOL.outputs());  % Calculate the control value
        VTOL.propagateDynamics(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawVTOL(VTOL.states);
    VTOL.states;
    dataPlot.updatePlots(t, ref_input, VTOL.states, u);
    observerPlot.updatePlots(t, VTOL.states, ctrl.x_hat_lat, ctrl.x_hat_lon);
end


