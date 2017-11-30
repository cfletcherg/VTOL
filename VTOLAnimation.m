classdef VTOLAnimation
    %
    %    Ballbeam animation
    %
    %--------------------------------
    properties
        target_handle
        box_handle
        motorR_handle
        motorL_handle
        
        radius
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = VTOLAnimation(P)
            
            
            
            figure(1), clf
            plot([0,0],[0,0],'k');
            hold on
            % initialize the ball and beam to initial conditions
            self=self.drawBox(P.z0, P.h0, P.theta0);
            self=self.drawTarget(P.z0);
            self=self.drawWingL(P.z0, P.h0, P.theta0);
            self=self.drawWingR(P.z0, P.h0, P.theta0);
            axis([-5, 5, -3, 7]);
        end
        %---------------------------
        function self=drawVTOL(self, x)
            % Draw ballbeam is the main function that will call the functions:
            % drawBall and drawBeam to create the animation.
            % x is the system state
            z = x(1);       % Horizontal position of ball, m
            h = x(2);
            theta = x(3);   % Angle of beam, rads
            
            self=self.drawBox(z, h, theta);
            self=self.drawWingL(z, h, theta);
            self=self.drawWingR(z, h, theta);
            self=self.drawTarget(z);
            drawnow
        end
        %---------------------------
        function self=drawBox(self, z, h, theta)
            
            % Put code here to draw your ball.
            % Save your data points into the X and Y vectors to draw below
            xbox = [-.2 .2 .2 -.2];
            ybox = [.2 .2 -.2 -.2];
            box = [xbox; ybox];
            boxrot = [cos(theta) -sin(theta);...
                sin(theta) cos(theta)]*box;
            X = boxrot(1,:) + z;
            Y = boxrot(2,:) + h;
            % this will only 'draw' the data points if needed, otherwise it
            % will just change the values in the handle.  It will still
            % update the animation, but is faster than a redraw).
            if isempty(self.box_handle)
                self.box_handle = fill(X, Y, 'r');
            else
                set(self.box_handle, 'XData', X, 'YData', Y);
            end
        end
        %---------------------------
        function self=drawWingL(self, z, h, theta)
            % Put code here to draw your beam.
            % Save your data points into the X and Y vectors to draw below
            r = .1;
            n = 1000;
            t = linspace(0,2*pi,n);
            c = [-.5 + r*sin(t); r*cos(t)];
            fanrot = [cos(theta) -sin(theta);...
                sin(theta) cos(theta)]*c;
            X = fanrot(1,:) + z;
            Y = fanrot(2,:) + h;
            % this will only 'draw' the data points if needed, otherwise it
            % will just change the values in the handle.  It will still
            % update the animation, but is faster than a redraw).
            if isempty(self.motorL_handle)
                self.motorL_handle = fill(X, Y, 'b');
            else
                set(self.motorL_handle,'XData', X, 'YData', Y);
            end
        end
        %---------------------------
        function self=drawWingR(self, z, h, theta)
            % Put code here to draw your beam.
            % Save your data points into the X and Y vectors to draw below
            r = .1;
            n = 1000;
            t = linspace(0,2*pi,n);
            c = [.5 + r*sin(t); r*cos(t)];
            fanrot = [cos(theta) -sin(theta);...
                sin(theta) cos(theta)]*c;
            X = fanrot(1,:) + z;
            Y = fanrot(2,:) + h;
            % this will only 'draw' the data points if needed, otherwise it
            % will just change the values in the handle.  It will still
            % update the animation, but is faster than a redraw).
            if isempty(self.motorR_handle)
                self.motorR_handle = fill(X, Y, 'b');
            else
                set(self.motorR_handle,'XData', X, 'YData', Y);
            end
        end
        %---------------------------
        function self=drawTarget(self, z)
            % Put code here to draw your beam.
            % Save your data points into the X and Y vectors to draw below
            X = [-.2 .2 .2 -.2];
            Y = [-.2 -.2 .2 .2];
            % this will only 'draw' the data points if needed, otherwise it
            % will just change the values in the handle.  It will still
            % update the animation, but is faster than a redraw).
            if isempty(self.target_handle)
                self.target_handle = fill(X, Y, 'b');
            else
                set(self.target_handle,'XData', X, 'YData', Y);
            end
        end
    end
end