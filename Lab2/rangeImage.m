classdef rangeImage

    methods(Static)
        function [ x, y, th] = irToXy( i, r )
        % irToXy finds position and bearing of a range pixel endpoint
        % Finds the position and bearing of the endpoint of a range pixel in
        % the plane.
            th1 = -5*(pi/360); %this needs 2 change
            th = th1 + (i-1)*(pi/180);
            if (th > pi)
                th = th-2*pi;
            end
            x = r*cos(th);
            y = r*sin(th);
        end
        
        function [] = followStraight(robot)
            idealObjectRange = 0.5;
            gain = .3;
            while true
                [x, y, ~] = rangeImage.findNearest(robot);
                dist = sqrt(x.^2 + y.^2);
                v = (dist-idealObjectRange)*gain;
                if ((dist == 0) || (abs(dist-idealObjectRange) < .07))
                    v = 0;
                end
                robot.sendVelocity(v, v);
                pause(0.3);
            end
        end
        
        function [x, y, th] = findNearest(robot)
           
           ranges = robot.laser.LatestMessage.Ranges;
           minDist = 1.0;
           x = 0;
           y = 0;
           th = 1;
           for i = 1:360 
               if ((ranges(i) < minDist) && (ranges(i) > 0.06))
                   [x1, y1, th1] = rangeImage.irToXy(i, ranges(i));
                   if (abs(th1) < (pi/2))
                       x = x1;
                       y = y1;
                       th = th1;
                       minDist = ranges(i);
                   end
               end
           end
           plot(-1:1,-1:1);
           hold on
           plot(x, y, 'x');
           
        end
        function [] = test()
            robot = raspbot('Raspbot-09');
            robot.startLaser();
            pause(4);
            %while true
            rangeImage.followStraight(robot);
            %    pause(0.2);
            %end
            robot.stopLaser();
        end
    end
end