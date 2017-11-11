classdef Lab11
    properties
        localizer; driver; robot; estRobot;
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
            
        function obj = Lab11()
            p1 = [0 ; 0];
            p2 = [ 0 ; 1.2192];
            p3 = [ 1.2192 ;  0];
            lines_p1 = [p1 p1];
            lines_p2 = [p2 p3];
            obj.localizer = lineMapLocalizer(lines_p1, lines_p2, 0.3, 0.004, 0.0005);
            obj.driver = robotKeypressDriver(gcf);
            
            obj.robot = raspbot('Raspbot-17')
            pause(2);
            
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            
            pause(3);
            encoderX = obj.robot.encoders.LatestMessage.Vector.X;
            encoderY = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.estRobot = simRobot1(encoderX, encoderY);
            
            pause(2);
            obj.lab11(); 
        end
        
        
        
        function lab11(obj)
            obj.robot.startLaser();
            pause(2);
            poseEst = pose(12*0.0254, 12*0.0254, pi/2.0);
            bodyPts = robotModel.bodyGraph();
            global currval;
            global preval;
            preval = false; currval = false;
            firstLoop = true;
            tcurr = 0;
            obj.estRobot.x = 12*0.0254;  obj.estRobot.y = 12*0.0254; obj.estRobot.theta = pi/2.0;
            while true
                if (firstLoop) 
                    firstLoop = false;
                    tStart = tic;
                    tcurr = 0;
                end
                while (preval == currval)
                   pause(0.001);
                end
                preval = currval;
                tcurr = toc(tStart);
                newx = obj.robot.encoders.LatestMessage.Vector.X;
                newy = obj.robot.encoders.LatestMessage.Vector.Y;
                obj.estRobot.integrate(newx, newy, tcurr);
                %step 1: odometry pose
                poseEst = pose(obj.estRobot.x, obj.estRobot.y, obj.estRobot.theta);
                
                
                %step 2: lidar pose 
                robotBodyPts = poseEst.bToA()*bodyPts;
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
                ids = obj.localizer.throwOutliers(poseEst, pointsInModelFrame);
            
                allIds = linspace(1, length(pointsInModelFrame), length(pointsInModelFrame));
                goodIds = setdiff(allIds, ids);
                pointsInModelFrame = pointsInModelFrame(:, goodIds);
                
                
                [success, poseLidar] = obj.localizer.refinePose(poseEst, pointsInModelFrame, 15, robotBodyPts);
                
                %step 3: new pest from pest and plid
                k = 0.25; % <= 1/4
                newTh = poseEst.th() + k*(poseLidar.th() - poseEst.th());
                newTh = atan2(sin(newTh),cos(newTh));
                newX = poseEst.x() + k*(poseLidar.x()-poseEst.x());
                newY = poseEst.y() + k*(poseLidar.y()-poseEst.y());
                
                poseEst = pose(newX,newY,newTh);
                
                obj.driver.drive(obj.robot, 2.0);
            end
        end
        
        
        
    end
end


function encoderEventListener(handle, event)
    global currval;
    currval = ~currval;  
end