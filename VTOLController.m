classdef VTOLController < handle
    %
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        hCtrl
        thetaCtrl
        ml
        mr
        mc
        g
        d
        Klon
        Klat
        beta
        tauMax
        Fmax
        Ts
        z0
        h0
        theta0
        zdot
        hdot
        thetadot
        kilat
        kilon
        integratorz
        integratorh
        error_dz
        error_dh
    end
    %----------------------------
    methods
        %----------------------------
        function self = VTOLController(P)
            self.ml = P.ml;
            self.mr = P.mr;
            self.mc = P.mc;
            self.d = P.d;
            self.g = P.g;
            self.Klon = P.Klon;
            self.Klat = P.Klat;
            self.beta = P.beta;
            self.tauMax = P.tauMax;
            self.Fmax = P.Fmax;
            self.Ts = P.Ts;
            self.z0 = P.z0;
            self.h0 = P.h0;
            self.theta0 = P.theta0;
            self.zdot = P.zdot0;
            self.hdot = P.hdot0;
            self.thetadot = P.thetadot0;
            self.error_dz = 0.0;
            self.error_dh = 0.0;
            self.kilat = P.kilat;
            self.kilon = P.kilon;
            self.integratorz = 0.0;
            self.integratorh = 0.0;
            
            % Instantiates the PD control objects
            % self.zCtrl = PIDControl(P.kp_z, P.kd_z, P.ki_z, P.theta_max, P.beta, P.Ts);
            % self.hCtrl = PIDControl(P.kp_h, P.kd_h, P.ki_h, P.Fmax, P.beta, P.Ts);
            % self.thetaCtrl = PIDControl(P.kp_th, P.kd_th, 0, P.tauMax, P.beta, P.Ts);
        end
        %----------------------------
        function u = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r(1);
            h_r = y_r(2);
            
            z = y(1);
            h = y(2);
            theta = y(3);
            
            
            Fe = (self.ml + self.mr + self.mc)*self.g;
            
            self.differentiateZ(z);
            self.differentiateH(h);
            self.differentiateTheta(theta);
            
            % integrate error
            errorh = h_r - h;
            errorz = z_r - z;
            self.integrateErrorH(errorh);
            self.integrateErrorZ(errorz);
            
            xlon = [h; self.hdot];
            F_tilda = -self.Klon*xlon - self.kilon*self.integratorh;
            
            xlat = [z; theta; self.zdot; self.thetadot];
            tau_unsat = -self.Klat*xlat - self.kilat*self.integratorz;
            %             Ftilda = self.hCtrl.PD(h_r, h, false);
            %             theta_r = self.zCtrl.PD(z_r, z, false);
            %             tau = self.thetaCtrl.PD(theta_r, theta, false);
            
            % Implement your controller here...
            
            % You may choose to implement the PD control directly or call the
            % PDControl class.  The PDControl class will return a force output
            % for the given reference input and current state.
            % i.e. for the z-controller (already set up in the constructor)
            % call: z_force = self.zCtrl.PD(z_r, z, false);
            % For the theta controller call:
            %       theta_force = self.thetaCtrl.PD(theta_r, theta, false);
            % You will need to determine what the output is for these
            % controllers in relation to the block diagrams derived for the
            % inner and outer loop control.
            
            % compute the total force
            F_unsat = Fe + F_tilda;
            F = self.saturateF(F_unsat);
            self.integratorAntiWindupF(F, F_unsat);
            tau = self.saturateTau(tau_unsat);
            self.integratorAntiWindupTau(tau, tau_unsat);
            u = [F; tau];
        end
        %----------------------------
        function self = integratorAntiWindupF(self, u_sat, u_unsat)
           self.integratorh = self.integratorh + self.Ts/self.kilon*(u_sat-u_unsat); 
        end
        %----------------------------
        function self = integratorAntiWindupTau(self, u_sat, u_unsat)
            self.integratorz = self.integratorz + self.Ts/self.kilat*(u_sat-u_unsat);
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.zdot = ...
                self.beta*self.zdot...
                + (1-self.beta)*((z-self.z0) / self.Ts);
            self.z0 = z;
        end
        %----------------------------
        function self = differentiateH(self, h)
            self.hdot = ...
                self.beta*self.hdot...
                + (1-self.beta)*((h-self.h0) / self.Ts);
            self.h0 = h;
        end
        %----------------------------
        function self = differentiateTheta(self, theta)
            self.thetadot = ...
                self.beta*self.thetadot...
                + (1-self.beta)*((theta-self.theta0) / self.Ts);
            self.theta0 = theta;
        end
        %----------------------------
        function out = saturateF(self,u)
            if abs(u) > self.Fmax
                u = self.Fmax*sign(u);
            end
            out = u;
        end
        %-------------------------
        function self = integrateErrorH(self, errorh)
            self.integratorh = self.integratorh + (self.Ts/2.0)*(errorh+self.error_dh);
            self.error_dh = errorh;
        end
        %-------------------------
        function self = integrateErrorZ(self, errorz)
            self.integratorz = self.integratorz + (self.Ts/2.0)*(errorz+self.error_dz);
            self.error_dz = errorz;
        end
        %----------------------------
        function out = saturateTau(self,u)
            if abs(u) > self.tauMax
                u = self.tauMax*sign(u);
            end
            out = u;
        end
    end
end


