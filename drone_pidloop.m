
function drone_pid = drone_pidloop(z_r,z,flag,kp,ki,kd,limit,Ts,sigma,vbar)
    persistent integrator;
    persistent differentiator;
    persistent error_d1;
    % reset (initialize) persistent variables when flag == 1
    if flag==1
        integrator = 0;
        differentiator = 0;
        error_d1 = 0; % _d1 means delayed by one time step
    end
    % compute the current error
    error = z_r - z;
    % update differentiator
    a1 = (2*sigma-Ts)/(2*sigma+Ts);
    a2 = 2/(2*sigma+Ts);
    differentiator = a1*differentiator+a2*(error-error_d1);
    % update integrator
    if abs(differentiator) < vbar
        integrator = integrator + (Ts/2)*(error + error_d1);
    end
    % unsaturated PID control
    u_unsat = kp*error + ki*integrator + kd*differentiator;
    % saturated PID control
    drone_pid = sat(u_unsat,limit);
    % anti-windup: update integrator to keep u out of saturation
    if ki ~= 0 % only implement when integrator is turned on
        integrator = integrator + 1/ki * (drone_pid - u_unsat);
    end
    % update the delayed error for next time through loop
    error_d1 = error;
end % end pidloop

% function to saturate the output
function out = sat(in, limit)
    if in > limit
        out = limit;
    elseif in < -limit
        out = -limit;
    else
        out = in;
    end
end % end sat