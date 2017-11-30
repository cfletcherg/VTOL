classdef VTOLDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        ml
        mr
        mc
        d
        Jc
        u
        g
        Ts
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = VTOLDynamics(P)
            % Initial state conditions
            self.state = [...
                        P.z0;...          % z initial position
                        P.h0;...          % h initial position
                        P.theta0;...      % Theta initial orientation
                        P.zdot0;...       % zdot initial velocity
                        P.hdot0;...       % hdot initial velicity
                        P.thetadot0;...   % Thetadot initial velocity
                        ];     
           
            self.ml = P.ml;  
            self.mr = P.mr;
            self.mc = P.mc;
            self.d = P.d;  
            self.Jc = P.Jc;
            self.u = P.u;
            self.g = P.g;
            self.Ts = P.Ts; 
          
        end
        %----------------------------
       function self = propagateDynamics(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            
            % Note: this is where you would make the call to propogate 
            % the dynamics using either Runge-Kutta or ODE45.  Both methods
            % will call the function self.derivatives.  In order to fit the
            % ODE45 method signature, the first parameter in the function
            % is the current time 't'.  This isn't used in either method,
            % but needs to be there.  If you use Runge-Kutta then you
            % should remove the 't' parameter and similarly remove the '0'
            % when the function gets called.
            
            %% Use either Runge-Kutta or ODE45 by uncommenting the 
            % corresponding code.  Do not use both.
            
            %Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(0,self.state, u);
            k2 = self.derivatives(0,self.state + self.Ts/2*k1, u);
            k3 = self.derivatives(0,self.state + self.Ts/2*k2, u);
            k4 = self.derivatives(0,self.state + self.Ts*k3, u);
            self.state = self.state + self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4);
            
            % Integrate ODE using ODE45 algorithm
            % [t xdot]= ode45(@self.derivatives,[0:self.Ts:self.Ts],self.state',[],u);
            % self.state = xdot(end,:)';
            
        end
        %----------------------------
        function xdot = derivatives(self,t, state, u)
            %
            % Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
            % 
            
            % Put your equations of motion here...
            z = state(1);
            h = state(2);
            theta = state(3);
            zdot = state(4);
            hdot = state(5);
            thetadot = state(6);
            
            zddot = (-u(1)*sin(theta)-self.u*zdot)/(self.mc+2*self.ml);
            thetaddot = (u(2)*self.d)/(2*self.d^2*self.ml+self.Jc);
            hddot = (u(1)*cos(theta)-self.mc*self.g-2*self.ml*self.g)/(self.mc+2*self.ml);
            
            xdot = [zdot; hdot; thetadot; zddot; hddot; thetaddot];

        end
        %----------------------------
        function y = outputs(self)
            %
            % Returns the measured outputs as a list
            % [z, theta] with added Gaussian noise
            % 
            z = self.state(1);
            h = self.state(2);
            theta = self.state(3);
            y = [z; h; theta];

        end
        %----------------------------
        function x = states(self)
            %
            % Returns all current states as a list
            %
            x = self.state;
        end
    end
end


