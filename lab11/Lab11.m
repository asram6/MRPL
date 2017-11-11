classdef Lab11
    properties
        localizer; driver; robot; estRobot; startPose; trajectoryObj;
        errorxarr; erroryarr ; errortharr; varr; warr; tarr; 
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
            
            obj.robot = raspbot('Raspbot-17');
            pause(2);
            
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            obj.robot.laser.NewMessageFcn = @laserEventListener;
            
            pause(3);
            encoderX = obj.robot.encoders.LatestMessage.Vector.X;
            encoderY = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.estRobot = simRobot1(encoderX, encoderY);
            obj.robot.startLaser();
            pause(2);
            obj.executeTrajectories(); 
        end
        
        function poseEst = updateStateFromEncodersAtTime(obj,newx,newy,tcurr, bodyPts)
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

                %obj.driver.drive(obj.robot, 2.0);
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
        
        function executeTrajectoryToAbsPose(obj,xfa,yfa,thfa,vmax,sgn,useMap, iteration)      
            % Execute a trajectory using state estimator as feedback      
            % Plan the trajectory. The terminal pose is specifie      
            % in world coordinates (not start relative). 
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(-xfa+obj.startPose(1), ...
                yfa-obj.startPose(2),...
                thfa-obj.startPose(3), sgn);
            obj.trajectoryObj.planVelocities(0.2);%0.2);
            obj.executeTrajectory(iteration);
        end
        
        
        function executeTrajectories(obj)
            obj.startPose = [0.6096;0.6096;pi()/2];
            %xf1 = 0.9144; yf1 = 0; thf1 = pi;
            xf1 = 0.3048; yf1 = 0.9144; thf1 = pi()/2.0;
            obj.executeTrajectoryToAbsPose(xf1, yf1, thf1, 0.2, 1, 1, 1);
            obj.startPose = [xf1; yf1; thf1];
            pause(2);
            xf2 = 0.9144; yf2 = 0.3048; thf2 = 0.0;
            obj.executeTrajectoryToAbsPose(xf2, yf2, thf2, 0.2, 1, 1, 2);
            pause(2);
            obj.startPose = [xf2; yf2; thf2];
            
            xf3 = 0.6096; yf3 = 0.6096; thf3 = pi()/2.0;
            xf3 = 1.2192; 
            obj.executeTrajectoryToAbsPose(xf3, yf3, thf3, 0.2, 1, 1, 3);
            pause(2);
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
        
        function executeTrajectory(obj, iteration)
                global currval; global currval2;
                global preval; global preval2;
                obj.errorxarr = []; obj.erroryarr = []; obj.errortharr = []; obj.varr = []; obj.warr = []; obj.tarr = [];
                preval = false; currval = false;
                preval2 = false; currval2 = false;
                trajectory = obj.trajectoryObj;
                firstLoop = true;
                %tStart = tic;
                tcurr = 0;
                xEnc = obj.robot.encoders.LatestMessage.Vector.X;
                yEnc = obj.robot.encoders.LatestMessage.Vector.Y;
                startX = xEnc;
                startY = yEnc;
                s = 0;
                referenceXArr = []; referenceYArr = []; sensedXArr = []; sensedYArr = [];
                finalPose = trajectory.getFinalPose();
                %figure(iteration); 
                %figure(iteration + 20); 
                %figure(iteration + 5);
                figure(100 + iteration);
                parms = trajectory.getParms();
                %fprintf("in executeTrajectory before loop \n");
                %lastI = size(trajectory.timeArray);
                lastT = trajectory.getTrajectoryDuration();%timeArray(lastI(2));
                tao = 0.4;%.27; %0.7;
                prevV = 0;
                error = 0;
                errory = 0;
                errorth = 0;
                tDelay = 0.115;%0.11;
                tcurr = 0;
                obj.estRobot.prevEncoderY = yEnc; %obj.estRobot.theta = 0;
                obj.estRobot.prevEncoderX = xEnc;
                obj.estRobot.tPrev = 0;
                referenceThArr = [];
                sensedThArr = [];
                pause(1);
                poseEst = pose(obj.startPose(1), obj.startPose(2), obj.startPose(3));
                newRangeImage = false;
                bodyPts = robotModel.bodyGraph();
                while (tcurr < lastT + tDelay) % || (errorth) > 0.02)
                    if (firstLoop) 
                        firstLoop = false;
                        tStart = tic;
                        tcurr = 0;
                        prevT = 0;
                    %else
                    %    prevT = tcurr;
                    %    tcurr = toc(tStart);
                    end
                    while (preval == currval)
                       pause(0.001);
                    end
                    encoderEventTime = double(obj.robot.encoders.LatestMessage.Header.Stamp.Sec) + double(obj.robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
                    preval = currval;
                    newxEnc = obj.robot.encoders.LatestMessage.Vector.X;
                    newyEnc = obj.robot.encoders.LatestMessage.Vector.Y;
                    
                    tcurr = toc(tStart);
                    prevT = tcurr;
                    if (tcurr < tDelay)
                        V = 0;
                        w = 0;
                    elseif (tcurr > lastT)
                        V = 0;
                        w = 0;
                    end
                    newRangeImage = false;
                    if (preval2 ~= currval2)
                        newRangeImage = true;
                        %fprintf("before %d \n", toc(tStart));
                        poseEst = obj.updateStateFromEncodersAtTime(newxEnc,newyEnc,tcurr, bodyPts);
                        %fprintf("after %d \n", toc(tStart));
                        preval2 = currval2;
                    end
                    
                    xEnc = newxEnc; yEnc = newyEnc;
                    sensedXArr = [sensedXArr x(poseEst)];
                    sensedYArr = [sensedYArr y(poseEst)];
                   
                    
                    if ((tcurr >= tDelay) && (tcurr <= lastT))
                        referencePose = trajectory.getPoseAtTime(tcurr-tDelay);
                        matrix = obj.bToA(obj.startPose)*obj.bToA(referencePose);
                        referencePose = obj.matToPoseVec(matrix);
                        prevV = trajectory.getVAtTime(tcurr-tDelay);
                        prevW = trajectory.getwAtTime(tcurr-tDelay);

                        errorx = referencePose(1) - poseEst.x(); errory = referencePose(2)-y(poseEst);
                        refTh = referencePose(3);
                        
                        %refTh = atan2(sin(refTh), cos(refTh));
                        refTh1 = mod(refTh,(2*pi));
                        sensedTheta = mod(th(poseEst), (2*pi));
                        errorth =  refTh1 - sensedTheta;
                        errorth = atan2(sin(errorth),cos(errorth));
                        
                        obj.errorxarr = [obj.errorxarr errorx];
                        obj.erroryarr = [obj.erroryarr errory];
                        obj.errortharr = [obj.errortharr errorth];
                        error = sqrt(errorx^2 + errory^2);
                        obj.tarr = [obj.tarr tcurr];
                        mat = zeros(2,2);%3,3);
                        mat(1,1) = cos(sensedTheta); mat(1,2) = -sin(sensedTheta); %mat(1,3) = x;
                        mat(2,1) = sin(sensedTheta); mat(2,2) = cos(sensedTheta); %mat(2,3) = y;
                        %mat(3,1) = 0.0; mat(3,2) = 0.0; mat(3, 3) = 1.0;
                        kth = 1/0.4; %1/0.2;%tao;
                        rpr = (mat^-1)*[errorx; errory];
                        thekx = 1/tao;
                    
                        %prevV = trajectory.getVAtTime(tcurr-tDelay);
                        theky = 2/(prevV*tao^2);
                        if (prevV < 0.05)
                            theky = 0;
                        end
                        if (newRangeImage == true)
                            up = [thekx*rpr(1); theky*rpr(2) + kth*errorth];
                        else
                            up = [0;0;0];
                        end
                        if (abs(errorth) < pi()/2 && abs(errorx) < 0.5 && abs(errory) < 0.5)
                            up = [0;0;0];
                        end
                        %prevW = trajectory.getwAtTime(tcurr-tDelay);
                        V = prevV + up(1);
                    
                        %prevW = atan2(sin(prevW), cos(prevW));
                        up2 = up(2);
                        %up2 = atan2(sin(up2), cos(up2));
                        w = prevW + up2;
                        w = atan2(sin(w),cos(w));
                        referenceXArr = [referenceXArr referencePose(1)];
                        referenceYArr = [referenceYArr referencePose(2)];
                        referenceThArr = [referenceThArr referencePose(3)];
                        sensedThArr = [sensedThArr sensedTheta];
                        obj.varr = [obj.varr V];
                        obj.warr = [obj.warr w];
                    end
                    
                    
                    %fprintf('V = %d w = %d\n', V, w);
                    [vl, vr]  = robotModel.VwTovlvr(V, w);
                    %fprintf('%d %d\n', vl,vr);
                    [vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
                    
                    obj.robot.sendVelocity(vl, vr);
                    pause(0.01);
                    
                end
                %obj.robot.sendVelocity(0,0);
                %pause(0.01);
                while (preval == currval)
                   pause(0.001);
                end
                
                encoderEventTime = double(obj.robot.encoders.LatestMessage.Header.Stamp.Sec) + double(obj.robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
                preval = currval;
                newxEnc = obj.robot.encoders.LatestMessage.Vector.X;
                newyEnc = obj.robot.encoders.LatestMessage.Vector.Y;
                %fprintf("diffx %d, diffy %d\n", newxEnc-startX, newyEnc-startY);
                tcurr = toc(tStart);
                obj.estRobot.integrate(newxEnc, newyEnc, tcurr);
                obj.robot.sendVelocity(0,0);
                %pause(2);
%                 if iteration ~= 1
%                     hold on;
%                 end
                %figure(10 + iteration);
                %plot(obj.tarr, referenceXArr, obj.tarr, referenceYArr, obj.tarr, referenceThArr);
                %legend("x", "y", "th");


                figure(100 + iteration);
                plot(referenceXArr, referenceYArr, sensedXArr, sensedYArr); 
                title("Reference Trajectory versus the Sensed Trajectory");
                xlabel('Position x (m)');
                ylabel('Position y (m)');
                %fprintf("error %d \n", sqrt((obj.sensedX-referencePose(1))^2 + (obj.sensedY-referencePose(2))^2));
                legend("reference", "sensed");
                hold on;
%                 if iteration ~= 1
%                     hold off;
%                 end 
                figure(iteration + 23);
                plot(obj.tarr, obj.errorxarr, obj.tarr, obj.erroryarr, obj.tarr, obj.errortharr);
                xlabel('time');
                ylabel('error');
                legend("x", "y", "th");
                title("Trajectory" + iteration);
                
                figure(13 + iteration);
                plot(obj.tarr, referenceThArr, obj.tarr,sensedThArr); 
                title("THETAAA");
                xlabel('Position x (m)');
                ylabel('Position y (m)');
                %fprintf("error %d \n", sqrt((obj.sensedX-referencePose(1))^2 + (obj.sensedY-referencePose(2))^2));
                legend("reference", "sensed");
                hold on;
                
                %figure(iteration + 5);
                %plot(obj.tarr, obj.varr, obj.tarr, obj.warr);
                %xlabel('time');
                %ylabel('velocity/ omega');
                %legend("v", "w");
                %title("Trajectory" + iteration);
%                 %plot(obj.warr, obj.errorxarr, obj.warr, obj.erroryarr, obj.warr, obj.errortharr);
        end
    end
        
    
end


function laserEventListener(handle, event)
    global currval2;
    currval2 = ~currval2;  
end


function encoderEventListener(handle, event)
    global currval;
    currval = ~currval;  
end