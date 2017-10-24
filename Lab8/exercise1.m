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
            robot = raspbot('Raspbot-17');
            pause(2);
            robot.startLaser();
            pause(4);
            ranges = robot.laser.LatestMessage.Ranges;           
            xArr = []; yArr = [];
            % remove all points with bad range
            goodOnes = ranges > 0.06 & ranges < 4.0;
            ranges = ranges(goodOnes);
            indices = linspace(1,length(goodOnes),length(goodOnes));
            indices = indices(goodOnes);
            % Compute the angles of surviving points
            for i = 1:length(indices) 
                   %[x1, y1, th1] = exercise1.irToXy(i, ranges(i));
                   [x1, y1, th1] = exercise1.irToXy(indices(i), ranges(i));
                   xArr = [xArr x1]; yArr = [yArr y1];
            end
            exercise1.findLineCandidate(ranges, xArr, yArr);
            scatter(xArr, yArr, 'g');
            
            
        robot.stopLaser();

        end
        
        function findLineCandidate(ranges, xArr, yArr)
            count = 0;  maxSetSize = 0;
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
                pointSetXCloud = pointSetX - centroidX; pointSetYCloud = pointSetY - centroidY;
                
                pointSetX1 = pointSetXCloud*centroidX; pointSetY1 = pointSetYCloud*centroidY;
                Ixx = sum(pointSetX1);
                Iyy = sum(pointSetY1);
                pointSetX2 = pointSetXCloud*(-1*centroidY);
                Ixy = sum(pointSetX2);
                Inertia = [Ixx Ixy;Ixy Iyy] / numPoints; % normalized
                lambda = eig(Inertia);
                lambda = sqrt(lambda)*1000.0;
                if ((numPoints >= 5) && (lambda(1) < 1.3) && numPoints > maxSetSize)
                    maxSetSize = numPoints;
                    topLeftX = min(pointSetX); topLeftY = max(pointSetY);
                    bottomRightX = max(pointSetX); bottomRightY = min(pointSetY);
                    diagonal = sqrt((bottomRightX - topLeftX)^2 + (bottomRightY - topLeftY)^2);
                    if abs(diagonal - 0.127) <= 0.01
                        pointSetX
                        pointSetY
                        count = count + 1;
                        fprintf("in here 4 numCount = %d\n", numPoints);
                        fprintf("topleftx %d, topLefty %d, bottomrightx %d, bottomrighy %d\n", topLeftX, topLeftY, bottomRightX, bottomRightY);
                        
                        orientation = atan2(2*Ixy,Iyy-Ixx)/2.0;
                        plot([topLeftX, bottomRightX], [topLeftY, bottomRightY]);
                        hold on;
                        
                        
                    end
                    
                end
                
            end
            fprintf("count %d\n", count);
            
             
        end
    end
end
