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
        
        function [] = follow(robot)
            idealObjectRange = 0.5;
            gain = .6;
            figure(1);
            clf;
            while true
                [x, y, th, dist] = rangeImage.findNearest(robot);
                %dist = sqrt(x.^2 + y.^2);
                v = (dist-idealObjectRange)*gain;
                if ((dist == 1.0) || (abs(dist-idealObjectRange) < .03))
                    v = 0;
                end
                vr = v + .0425*(th);
                vl = v - .0425*(th);
                robot.sendVelocity(vr, vl);
                pause(0.5);
                plot(-y, x, 'x');
                axis([-2 2 -2 2]);
                title('Closest point in robot frame');
                xlabel('X (m)');
                ylabel('Y (m)');
            end
        end
        
        function [x, y, th, minDist] = findNearest(robot)
           
           ranges = robot.laser.LatestMessage.Ranges;
           minDist = 1.0;
           x = 0;
           y = 0;
           th = 0;
           for i = 1:360 
               if ((ranges(i) < minDist) && (ranges(i) > 0.1))
                   [x1, y1, th1] = rangeImage.irToXy(i, ranges(i));
                   if (abs(th1) < (pi/2))
                       x = x1;
                       y = y1;
                       th = th1;
                       minDist = ranges(i);
                   end
               end
           end
           
        end
        function [] = test()
            robot = raspbot('Raspbot-09');
            robot.startLaser();
            pause(4);
            rangeImage.follow(robot);
            robot.stopLaser();
        end
    end
end