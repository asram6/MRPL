classdef exercise1
    methods(Static = true)
        function [ x, y, th] = irToXy( i, r )
                % irToXy finds position and bearing of a range pixel endpoint
                % Finds the position and bearing of the endpoint of a range pixel in
                % the plane.
                    %th1 = -5*(pi/360); %this needs 2 change
                    thOffset = atan2(0.0087,0.993);
                    th = (i-1)*(pi/180)-thOffset;
                    if (th > pi)
                        th = th-2*pi;
                    end
                    x = r*cos(th);
                    y = r*sin(th);
        end

        function exerciseOne()
            figure(1);
            robot = raspbot('robot');
            pause(2);
            robot.startLaser();
            pause(4);
            totalranges = [];
            ranges = robot.laser.LatestMessage.Ranges;
            pause(10);
            x = 0;
            y = 0;
            th = 0;
            xArr = []; yArr = [];
            % remove all points with bad range
            goodOnes = ranges > 0.06 & ranges < 4.0;
            ranges = ranges(goodOnes);
            %indices = linspace(1,length(ranges),length(ranges));
            %indices = indices(goodOnes);
            % Compute the angles of surviving points
            for i = 1:length(ranges) 
                   [x1, y1, th1] = exercise1.irToXy(i, ranges(i));
                   xArr = [xArr x1]; yArr = [yArr y1];
            end
            exercise1.findLineCandidate(ranges, xArr, yArr);
            figure(1);
            plot(xArr, yArr);
            
        robot.stopLaser();

        end
        
        function findLineCandidate(ranges, xArr, yArr)
            for i = 1:length(ranges)
                pointSetX = []; pointSetY = [];
                x = xArr(i); y = yArr(i);
                for j = 1:length(ranges)
                    otherX = xArr(j); otherY = yArr(j);
                    dist = sqrt((otherX - x)^2 + (otherY - y)^2);
                    if dist <= 0.0635
                        pointSetX = [pointSetX otherX];
                        pointSetY = [pointSetY otherY];
                    end
                end
                numPoints = length(pointSetX);
                centroidX = mean(pointSetX); centroidY = mean(pointSetY); 
                pointSetX = pointSetX - centroidX; pointSetY = pointSetY - centroidY;
                pointSetX1 = pointSetX*centroidX; pointSetY1 = pointSetY*centroidY;
                Ixx = sum(pointSetX1);
                Iyy = sum(pointSetY1);
                pointSetX2 = pointSetX*(-1*centroidY);
                Ixy = sum(pointSetX2);
                Inertia = [Ixx Ixy;Ixy Iyy] / numPoints; % normalized
                lambda = eig(Inertia);
                lambda = sqrt(lambda)*1000.0;
                if ((numPoints >= 5) && (lambda(1) < 1.3))
                    topLeftX = min(pointSetX); topLeftY = max(pointSetY);
                    bottomRightX = max(pointSetX); bottomRightY = min(pointSetY);
                    diagonal = sqrt((bottomRightX - topLeftX)^2 + (bottomRightY - topLeftY)^2);
                    if abs(diagonal - 0.127) <= 0.1
                        orientation = atan2(2*Ixy,Iyy-Ixx)/2.0;
                        figure(1);
                        plot(topLeftX, topLeftY, bottomRightX, bottomRightY);
                    end
                    
                end
                
            end
            
             
        end
    end
end
