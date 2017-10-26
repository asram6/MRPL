classdef exercise1
    
    properties
        robot, mrplSystem;
    end
    
    methods
        function obj = exercise1()
            obj.robot = raspbot('Raspbot-16');
            obj.mrplSystem = mrplSystemLab8(obj.robot)
            obj.exerciseOne();
        end
    
 
    
        function [ x, y, th] = irToXy( obj, i, r )
                % irToXy finds position and bearing of a range pixel endpoint
                % Finds the position and bearing of the endpoint of a range pixel in
                % the plane.
                    %th1 = -5*(pi/360); %this needs 2 change
                    thOffset = 0.0872665 - 0.0596773;
                    %thOffset = .05236; %%atan2(0.087,0.993)+;
                    th = (i-1)*(pi/180)-thOffset;
                    if (th > pi)
                        th = th-2*pi;
                    end
                    x = r*cos(th);
                    y = r*sin(th);
        end

        function exerciseOne(obj)
            figure(1);
            
            pause(2);
            obj.robot.startLaser();
            pause(4);
            ranges = obj.robot.laser.LatestMessage.Ranges;           
            xArr = []; yArr = [];
            % remove all points with bad range
            goodOnes = ranges > 0.06; %ranges > 0.06 & ranges < 4.0;
            ranges = ranges(goodOnes);
            indices = linspace(1,length(goodOnes),length(goodOnes));
            indices = indices(goodOnes);
            % Compute the angles of surviving points
            for i = 1:length(indices) 
                   %[x1, y1, th1] = exercise1.irToXy(i, ranges(i));
                   [x1, y1, th1] = obj.irToXy(indices(i), ranges(i));
                   xArr = [xArr x1]; yArr = [yArr y1];
            end
            offset = -0.14; %raise if doing two measurements
            obj.findLineCandidate(ranges, xArr, yArr, offset);
            scatter(xArr, yArr, 'g');
            %obj.robot.stopLaser();
            pause(1);
            
            
%             xArr = []; yArr = [];
%             % remove all points with bad range
%             goodOnes = ranges > 0.06; %ranges > 0.06 & ranges < 4.0;
%             ranges = ranges(goodOnes);
%             indices = linspace(1,length(goodOnes),length(goodOnes));
%             indices = indices(goodOnes);
%             % Compute the angles of surviving points
%             for i = 1:length(indices) 
%                    %[x1, y1, th1] = exercise1.irToXy(i, ranges(i));
%                    [x1, y1, th1] = obj.irToXy(indices(i), ranges(i));
%                    xArr = [xArr x1]; yArr = [yArr y1];
%             end
%             obj.findLineCandidate(ranges, xArr, yArr, -0.12);
%             scatter(xArr, yArr, 'g');

            obj.robot.stopLaser();             
            obj.robot.sendVelocity(.1,.1);
            pause(.6);
            obj.robot.sendVelocity(0,0);
            pause(2);
            obj.robot.forksUp();
            pause(3);
        end
        
        function [ranges, xArr, yArr] = getTestData()
            figure(1);
            robot = raspbot('Raspbot-17');
            pause(2);
            robot.startLaser();
            pause(4);
            ranges = robot.laser.LatestMessage.Ranges;           
            xArr = []; yArr = [];
            % remove all points with bad range
            goodOnes = ranges > 0.06; %ranges > 0.06 & ranges < 4.0;
            ranges = ranges(goodOnes);
            indices = linspace(1,length(goodOnes),length(goodOnes));
            indices = indices(goodOnes);
            % Compute the angles of surviving points
            for i = 1:length(indices) 
                   %[x1, y1, th1] = exercise1.irToXy(i, ranges(i));
                   [x1, y1, th1] = exercise1.irToXy(indices(i), ranges(i));
                   xArr = [xArr x1]; yArr = [yArr y1];
            end
            obj.robot.stopLaser();
        end
        
        function findLineCandidate(obj, ranges, xArr, yArr, offset)
            scatter(xArr, yArr, 'g');
            centroidXFinal = 0; centroidYFinal = 0; orientationFinal = 0; found = false;
            hold on;
            count = 0;  minDistance = 4.0; maxPoints = 0;
            for i = 1:length(ranges)
                pointSetX = []; pointSetY = [];
                x = xArr(i); y = yArr(i);
                for j = 1:length(ranges)
                    otherX = xArr(j); otherY = yArr(j);
                    dist = sqrt((otherX - x)^2 + (otherY - y)^2);
                    threshold = 0.01;
                    if dist <= 0.0635 + threshold;
                        pointSetX = [pointSetX otherX];
                        pointSetY = [pointSetY otherY];
                    end
                end
                numPoints = length(pointSetX);
                centroidX = mean(pointSetX); centroidY = mean(pointSetY); 
                distance = sqrt(centroidX^2 + centroidY^2);
                pointSetXCloud = pointSetX - centroidX; pointSetYCloud = pointSetY - centroidY;
                
                %pointSetX1 = pointSetXCloud*centroidX; pointSetY1 = pointSetYCloud*centroidY;
                %Ixx = sum(pointSetX1); %%%%SOMETHINS UP
                %Iyy = sum(pointSetY1);
                %pointSetX2 = pointSetXCloud*(-1*centroidY);
                %Ixy = sum(pointSetX2);
                Ixx = sum(pointSetXCloud.^2);
                Iyy = sum(pointSetYCloud.^2);
                Ixy = -1*sum(pointSetXCloud.*pointSetYCloud);
                Inertia = [Ixx Ixy;Ixy Iyy] / numPoints; % normalized
                lambda = eig(Inertia);
                lambda = sqrt(lambda)*1000.0;
                distanceThreshold = 0.01;
                if ((numPoints >= 7) && (lambda(1) < 1.3) && (numPoints >= maxPoints) && (distance < minDistance + distanceThreshold))
                    leftX = min(pointSetX); topY = max(pointSetY);
                    rightX = max(pointSetX); bottomY = min(pointSetY);
                    diagonal = sqrt((rightX - leftX)^2 + (bottomY - topY)^2);
                    if abs(diagonal - 0.127) <= 0.035
                        found = true;
                        maxPoints = numPoints;
                        minDistance = distance;
                        count = count + 1;
                        fprintf("in here 4 numCount = %d\n", numPoints);
                        fprintf("leftx %d, topy %d, rightx %d, bottomy %d\n", leftX, topY, rightX, bottomY);
                        
                        orientation = atan2(2*Ixy,Iyy-Ixx)/2.0;
                        centroidXFinal = centroidX; centroidYFinal = centroidY; 
                        orientationFinal = orientation;
                        %orientation = atan2(sin(orientation), cos(orientation));
                        %fprintf("%d\n", orientation);
                        fprintf("orientation %d, centroidX %d, centroidY %d,\n", orientation, centroidX, centroidY);
                        
                        if (((orientation < 0)))% && (centroidX > 0)) || ((orientation >= 0) && (centroidX <= 0))) 
                            plot([leftX, rightX], [bottomY, topY]);
                        else
                            plot([leftX, rightX], [topY, bottomY]);
                        end
                        hold on;
                        plot(centroidX, centroidY, 'r*');
                        
                        hold on;
                        
                        
                    end
                    
                end
                
            end
            
            if (found)
                fprintf("orientation final %d\n", orientationFinal);
                 Trs = [0;0;0];
                 Tso = [centroidXFinal, centroidYFinal, orientation];
                 %good was 0.12/ -.25 for first, -.12 for second
                 Tog = [offset; 0; 0];
                 matrix = obj.bToA(Trs)*obj.bToA(Tso)*obj.bToA(Tog);
                 newPose = obj.matToPoseVec(matrix);
                %fprintf("the pose x %d, y %d, th %d\n", newPose(1), newPose(2), newPose(3));
                obj.robot.forksDown();
                pause(2);
                obj.mrplSystem.executeTrajectoryLab8(newPose);
                
                %otherX = centroidXFinal + 10*cos(-1*orientation); otherY = centroidYFinal + 10*sin(-1*orientation);
                %otherX2 = centroidXFinal + 10*cos(orientation); otherY2 = centroidYFinal + 10*sin(orientation);
                %plot([centroidXFinal otherX], [centroidYFinal, otherY]);
                %hold on;
                plot([centroidXFinal newPose(1)], [centroidYFinal, newPose(2)]);
                hold on;
                %tstart = tic; t = toc(tstart);
                %while t < 0.0001
                %    obj.robot.sendVelocity(0.05,0.05);
                %    t = toc(tstart);
                %end
            end
            
            fprintf("count %d\n", count);
            
             
        end
    
        function mat = bToA(obj, pose)
            % Returns the homogeneous transform that converts coordinates from
            % the b frame to the a frame.

            mat = zeros(3,3);
            x = pose(1); y = pose(2); th = pose(3);

            mat(1,1) =  cos(th); mat(1,2) = -sin(th); mat(1,3) = x;
            mat(2,1) =  sin(th); mat(2,2) =  cos(th); mat(2,3) = y;
            mat(3,1) =  0.0    ; mat(3,2) =  0.0    ; mat(3,3) = 1;
        end
        
        function vec = matToPoseVec(obj, mat)
            % Convert a homogeneous transform into a vector that can be
            % passed to the contructor for this class.
            x = mat(1,3);
            y = mat(2,3);
            w = atan2(-mat(1,2),mat(1,1));
            vec = [x ; y ; w];
        end
    end
end

