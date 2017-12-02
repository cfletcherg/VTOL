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
        x_hat_lat
        x_hat_lon
        Alat
        Blat
        Clat
        Llat
        Alon
        Blon
        Clon
        Llon
        F_d1
        Tau_d1
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
            self.error_dz = 0;
            self.error_dh = 0;
            self.kilat = P.kilat;
            self.kilon = P.kilon;
            self.integratorz = 0;
            self.integratorh = 0;
            self.Alat = P.Alat;
            self.Blat = P.Blat;
            self.Clat = P.Clat;
            self.Llat = P.Llat;
            self.Alon = P.Alon;
            self.Blon = P.Blon;
            self.Clon = P.Clon;
            self.Llon = P.Llon;
            self.F_d1 = 0;
            self.Tau_d1 = 0;
            self.x_hat_lat = [0; 0; 0; 0];
            self.x_hat_lon = [0; 0];
            
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
            ylat = [y(1); y(3)];
            ylon = [y(2)];
            
            % update the observer and extract z_hat and h_hat
            self.updateObserverLat(ylat);
            self.updateObserverLon(ylon);
            z_hat = self.x_hat_lat(1);
            h_hat = self.x_hat_lon(1);
            
            Fe = (self.ml + self.mr + self.mc)*self.g;
            
            % integrate error
            errorh = h_r - h_hat;
            errorz = z_r - z_hat;
            self.integrateErrorH(errorh);
            self.integrateErrorZ(errorz);

            F_tilda = -self.Klon*self.x_hat_lon - self.kilon*self.integratorh;
            
            tau_unsat = -self.Klat*self.x_hat_lat - self.kilat*self.integratorz;
           
            F_unsat = Fe + F_tilda;
            F = self.saturateF(F_unsat);
            self.updateForce(F);
            tau = self.saturateTau(tau_unsat);
            self.updateTau(tau);
            u = [F; tau];
        end
        %----------------------------
        function self = updateForce(self, F)
            self.F_d1 = F;
        end
        %----------------------------
        function self = updateTau(self, tau)
            self.Tau_d1 = tau;
        end
        %----------------------------
        function self = updateObserverLat(self, y_m)
            % compute equilibrium torque tau_e at old angle
            Tau_e = 0;

            N = 10;
            for i=1:N
                self.x_hat_lat = self.x_hat_lat + self.Ts/N*(...
                    self.Alat*self.x_hat_lat...
                    + self.Blat*(self.Tau_d1-Tau_e)...
                    + self.Llat*(y_m-self.Clat*self.x_hat_lat));
            end
        end
        %----------------------------
        function self = updateObserverLon(self, y_m)
            % compute equilibrium torque tau_e at old angle
            Fe = (self.ml + self.mr + self.mc)*self.g;

            N = 10;
            for i=1:N
                self.x_hat_lon = self.x_hat_lon + self.Ts/N*(...
                    self.Alon*self.x_hat_lon...
                    + self.Blon*(self.F_d1-Fe)...
                    + self.Llon*(y_m-self.Clon*self.x_hat_lon));
            end
        end
        %----------------------------
%         function self = integratorAntiWindupF(self, u_sat, u_unsat)
%            self.integratorh = self.integratorh + self.Ts/self.kilon*(u_sat-u_unsat); 
%         end
        %----------------------------
%         function self = integratorAntiWindupTau(self, u_sat, u_unsat)
%             self.integratorz = self.integratorz + self.Ts/self.kilat*(u_sat-u_unsat);
%         end
        %----------------------------
%         function self = differentiateZ(self, z)
%             self.zdot = ...
%                 self.beta*self.zdot...
%                 + (1-self.beta)*((z-self.z0) / self.Ts);
%             self.z0 = z;
%         end
%         %----------------------------
%         function self = differentiateH(self, h)
%             self.hdot = ...
%                 self.beta*self.hdot...
%                 + (1-self.beta)*((h-self.h0) / self.Ts);
%             self.h0 = h;
%         end
%         %----------------------------
%         function self = differentiateTheta(self, theta)
%             self.thetadot = ...
%                 self.beta*self.thetadot...
%                 + (1-self.beta)*((theta-self.theta0) / self.Ts);
%             self.theta0 = theta;
%         end
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


