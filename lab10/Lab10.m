classdef Lab10
    properties
        localizer; driver; robot;
    end
    
    methods
        
        function [ x, y, th] = irToXy( obj, i, r )
                % irToXy finds position and bearing of a range pixel endpoint
                % Finds the position and bearing of the endpoint of a range pixel in
                % the plane.
                    %th1 = -5*(pi/360); %this needs 2 change
                   % thOffset = 0.0872665 - 0.0596773;
                    %thOffset = .05236; %%atan2(0.087,0.993)+;
                    thOffset = 0.07;
                    th = (i-1)*(pi/180)-thOffset;
                    if (th > pi)
                        th = th-2*pi;
                    end
                    x = r*cos(th);
                    y = r*sin(th);
        end
            
        function obj = Lab10()
            p1 = [0 ; 0];
            p2 = [ 0 ; 1.2192];
            p3 = [ 1.2192 ;  0];
            lines_p1 = [p1 p1];
            lines_p2 = [p2 p3];
            obj.localizer = lineMapLocalizer(lines_p1, lines_p2, 0.3, 0.004, 0.0005);
            obj.driver = robotKeypressDriver(gcf);
            obj.robot = raspbot('Raspbot-37');
            pause(2);
            obj.lab10();
            
        end
        
        function lab10(obj)
            obj.robot.startLaser();
            pause(2);
            currPose = pose(12*0.0254, 12*0.0254, pi/2.0);
            bodyPts = robotModel.bodyGraph();

            while true
                robotBodyPts = currPose.bToA()*bodyPts;
                pts = obj.robot.laser.LatestMessage.Ranges;
                xArr = []; yArr = []; thArr = []; wArr = [];
                for i = 1:length(pts)
                    if (mod(i, 10) == 0)
                        [x,y,th] = obj.irToXy(i, pts(i));
                        xArr = [xArr x];
                        yArr = [yArr y];
                        wArr = [wArr 1.0];
                    end
                end
                pointsInModelFrame = [xArr ; yArr; wArr];
                ids = obj.localizer.throwOutliers(currPose, pointsInModelFrame);
            
                allIds = linspace(1, length(pointsInModelFrame), length(pointsInModelFrame));
                goodIds = setdiff(allIds, ids);
                pointsInModelFrame = pointsInModelFrame(:, goodIds);
                [success, currPose] = obj.localizer.refinePose(currPose, pointsInModelFrame, 15, robotBodyPts);
                %fprintf("pose %d, %d, %d\n", currPose.x(),currPose.y(), currPose.th());
                %obj.robot.sendVelocity(.04, .04);
                obj.robot.sendVelocity(.0212, .02);
                pause(0.2);
            end
        end
        
        
        
    end
end