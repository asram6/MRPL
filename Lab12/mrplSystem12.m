classdef mrplSystem12 < handle
    properties
        localizer; driver; robot; estRobot; startPose; trajectoryObj;
        errorxarr; erroryarr ; errortharr; varr; warr; tarr; 
        odometryX;odometryY; lidarX; lidarY; sensedX; sensedY; sensedTh;
    end
    
    methods
       	function mat = aToB(obj, pose)
            % Returns the homogeneous transform that converts coordinates from
            % the a frame to the b frame.

            bTa = obj.bToA(pose);

            mat = bTa^-1;
        end
        
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
            
        function obj = mrplSystem12()
            obj.odometryX = [];obj.odometryY = []; obj.lidarX = []; obj.lidarY = [];
            p1 = [0 ; 0];
            p2 = [ 0 ; 1.2192];
            p3 = [ 1.2192 ;  0];
            p4 = [1.2192; 1.2192];
            lines_p1 = [p1 p1 p3];
            lines_p2 = [p2 p3 p4];
            lines_p3 = [p1 p3];
            lines_p4 = [p3 p4];
            obj.localizer = lineMapLocalizer12(lines_p1, lines_p2, 0.3, 0.01, 0.0005);%0.004, 0.0005);
            %obj.localizerLeft = lineMapLocalizer12(lines_p1, lines_p2, 0.3, 0.01, 0.0005);%0.004, 0.0005);
            %obj.localizerRight = lineMapLocalizer12(lines_p3, lines_p4, 0.3, 0.01, 0.0005);%0.004, 0.0005);
            obj.driver = robotKeypressDriver(gcf);
            
            obj.robot = raspbot('Raspbot-32');
            pause(2);
            
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            obj.robot.laser.NewMessageFcn = @laserEventListener;
            
            pause(3);
            encoderX = obj.robot.encoders.LatestMessage.Vector.X;
            encoderY = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.estRobot = simRobot1(encoderX, encoderY);
            obj.robot.startLaser();
            pause(2);
            obj.lab12ChallengeTask(); 
        end
        
        function poseEst = updateStateFromEncodersAtTime(obj,newx,newy,tcurr, bodyPts, flag)
            obj.estRobot.integrate(newx, newy, tcurr);
            %step 1: odometry pose
           
            poseEst = pose(obj.estRobot.x, obj.estRobot.y, obj.estRobot.theta);
            obj.odometryX = [obj.odometryX, obj.estRobot.x];
            obj.odometryY = [obj.odometryY, obj.estRobot.y];
%             fprintf("IN UPDATE STATE START \n");
%             obj.odometryX
%             obj.estRobot.x
%             obj.odometryY
%             obj.estRobot.y
%             fprintf("IN UPDATE STATE END \n");
           % if (
            if (~flag)
                %step 2: lidar pose 
                robotBodyPts = poseEst.bToA()*bodyPts;
                pts = obj.robot.laser.LatestMessage.Ranges;
                xArr = []; yArr = []; thArr = []; wArr = [];
                for i = 1:length(pts)
                    if (mod(i, 5) == 0)
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


                [success, poseLidar] = obj.localizer.refinePose(poseEst, pointsInModelFrame, 11, robotBodyPts);

                %step 3: new pest from pest and plid
                k = 0.25;%0.25; % <= 1/4
                newTh = poseEst.th() + k*(poseLidar.th() - poseEst.th());
                newTh = atan2(sin(newTh),cos(newTh));
                newX = poseEst.x() + k*(poseLidar.x()-poseEst.x());
                newY = poseEst.y() + k*(poseLidar.y()-poseEst.y());
                obj.lidarX = [obj.lidarX poseLidar.x()];
                obj.lidarY = [obj.lidarY poseLidar.y()];
                poseEst = pose(newX,newY,newTh);
                obj.sensedX = newX; obj.sensedY = newY; obj.sensedTh = newTh;
                    %obj.driver.drive(obj.robot, 2.0);
            end
        end
            
       
        function updateSensedState(obj, tcurr)
            bodyPts = robotModel.bodyGraph();                
            newxEnc = obj.robot.encoders.LatestMessage.Vector.X;
            newyEnc = obj.robot.encoders.LatestMessage.Vector.Y;
           % tStart = tic;
           % tcurr = toc(tStart);
            obj.updateStateFromEncodersAtTime(newxEnc, newyEnc, tcurr, bodyPts, false);
        end
            
        function moveRelDist(obj,dist, doControlPlotting)
            % move forward or backward a specified distance and stop.
            % make sure the velocity is such that the distance will take at
            % least a second
            sgn = 1;
            if dist < 0
                sgn = -1;
            end
            
            obj.executeTrajectoryToRelativePose(abs(dist), 0, 0, sgn, 1);
            %obj.startPose = [abs(dist); 0; 0];
        end
        
        function turnRelAngle(obj, angle, doControlPlotting)
            % make sure the velocity is such that the distance will take at
            % least a secondFill me in
            % move a distance forward or backward
            seconds = 3; %1.5;
            %angle = angle/2;
            w = angle/seconds;
            V = 0;
            tstart = tic;
            
            tcurr = toc(tstart);
            obj.estRobot.tPrev = tcurr;
            while (toc(tstart) < seconds)
                [vl, vr]  = robotModel.VwTovlvr(V, w);
                [vl, vr] = robotModel.limitWheelVelocities([vl, vr]); 
                obj.robot.sendVelocity(vl, vr);
                pause(0.5);
            end
            obj.robot.sendVelocity(0, 0);
            pause(0.1);
            tcurr = toc(tstart);
            obj.updateSensedState(tcurr);
        end
        
        function executeTrajectoryToRelativePose(obj, x, y, th, sgn, iteration)
            fprintf("x %d   y %d   th %d \n", x, y, th);
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            obj.trajectoryObj.planVelocities(0.1);
            obj.executeTrajectory(iteration);
        end
        
        function lab12ChallengeTask(obj)
            %pickPoses = [[0.3048, 1.0668, pi/2]; [0.6098, 1.0688, pi/2]; [0.9144, 1.0688, pi/2]]; 
            %dropPoses = [[0.5344, .1524, -pi/2]; [0.6858, 0.1524, -pi/2]; [0.8382, 0.1524, -pi/2]];
            pickPose1 = [0.3048, 1.0668, pi/2]; pickPose2 = [0.6096, 1.0668, pi/2]; pickPose3 = [0.9144, 1.0668, pi/2];
            dropPose1 = [0.5344, .1524, -pi/2]; dropPose2 = [0.6858, 0.1524, -pi/2]; dropPose3 = [0.8382, 0.1524, -pi/2];
            
            obj.startPose = [0.2286; 0.2286; pi()/2];
            obj.sensedX = obj.startPose(1); obj.sensedY = obj.startPose(2); obj.sensedTh = obj.startPose(3);
            obj.estRobot.x = 0.2286; obj.estRobot.y = 0.2286; obj.estRobot.theta = pi()/2;
            %obj.pickDropObject(pickPose1, dropPose1);
            
            obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
            obj.pickDropObject(pickPose2, dropPose2);
            
            obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
            obj.pickDropObject(pickPose3, dropPose3);
        end
        
        
        function goToPickUpPose(obj, relDestPose)
            obj.robot.forksDown();
            pause(0.2);
            offset = -0.06;
            Tro = relDestPose;
            Tog = [offset, 0, 0];
            pickUpPose = obj.matToPoseVec(obj.bToA(Tro)*obj.bToA(Tog));
            %obj.executeTrajectoryToRelativePose(pickUpPose);
            fprintf("IN PICK UP POSE %d \n", atan2(pickUpPose(2), pickUpPose(1)));
%             fprintf("IN PICK UP POSE rel dest %d \n", atan2(relDestPose(2), relDestPose(1)));
%             fprintf("sendsed %d %d %d  \n", obj.sensedX, obj.sensedY, obj.sensedTh);
%             fprintf("relDest %d %d %d  \n", relDestPose(1), relDestPose(2), relDestPose(3));
            absPick = obj.relToAbs(pickUpPose);
%             fprintf("abs pickup %d %d %d \n", absPick(1),absPick(2),absPick(3));
            %obj.relToAbs(pickUpPose);
            obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
            
            %obj.executeTrajectoryToRelativePose(relDestPose(1) - 0.06, relDestPose(2), relDestPose(3), 1, 1);
            turnAngle = atan2(relDestPose(2), relDestPose(1));
            %turnThreshold = pi/8;
            %if (turnAngle > turnThreshold) %can change angle threshold
            obj.turnRelAngle(turnAngle, 0); %test to see if negative
            %end
            fprintf("HEY %d \n", pickUpPose(1));
            obj.moveRelDist(pickUpPose(1));
            obj.robot.forksUp();
            pause(0.2);
        end
        
        function goToDropPose(obj, absDestPose)
            offset = -0.12;
            relDestPose = obj.absToRel(absDestPose);
            Tro = relDestPose;
            Tog = [offset, 0, 0];
            dropPose = obj.matToPoseVec(obj.bToA(Tro)*obj.bToA(Tog));
            absDropPose = obj.relToAbs(dropPose);
            
            obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
            angle = obj.sensedTh - absDestPose(3);
            fprintf("absDestPose %d   relDesetPose %d   sensedTh %d \n",  absDestPose(3), relDestPose(3), obj.sensedTh);
            fprintf("angle %d  \n", atan2(sin(angle), cos(angle)));
            obj.turnRelAngle(atan2(sin(angle), cos(angle)));
                        
            obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
            relDestPose = obj.absToRel(absDestPose);
            
            fprintf("rel drop pose %d %d %d \n", relDestPose(1), relDestPose(2), relDestPose(3));
            fprintf("abs drop pose %d %d %d \n", absDropPose(1), absDropPose(2), absDropPose(3));
            obj.executeTrajectoryToAbsPose(absDropPose(1),absDropPose(2),absDropPose(3),0.1, 1, 1, 1);
            obj.robot.forksDown();
            pause(0.2);
            obj.moveRelDist(-0.07, 1);
            pause(0.1);
        end
        
        function pickDropObject(obj, absPickPose, absDropPose)
            Tro = obj.absToRel(absPickPose);
            fprintf("abs pick pose %d %d %d \n", absPickPose(1), absPickPose(2), absPickPose(3));
            fprintf("tro pose %d %d %d \n", Tro(1), Tro(2), Tro(3));

            offset = -0.24;%-0.14;
            Tog = [offset, 0, 0];
            
            acqPose = obj.matToPoseVec(obj.bToA(Tro)*obj.bToA(Tog)); %acquisition pose, can offset more
            absAcqPose = obj.relToAbs(acqPose);
            fprintf("acq pose %d %d %d \n", acqPose(1), acqPose(2), acqPose(3));
            currPose = [obj.sensedX, obj.sensedY, obj.sensedTh];
            turnAngle = acqPose(3);
            turnThreshold = pi/8;
            if (turnAngle > turnThreshold) %can change angle threshold
                obj.turnRelAngle(turnAngle, 0); %test to see if negative
            end
            relAcqPose = obj.absToRel(absAcqPose);
            fprintf("current pose %d %d %d \n", obj.sensedX, obj.sensedY, obj.sensedTh);
            fprintf("rel acq pose %d %d %d \n", relAcqPose(1), relAcqPose(2), relAcqPose(3));
            fprintf("abs acq pose %d %d %d \n", absAcqPose(1), absAcqPose(2), absAcqPose(3));
            vmax = 0.1;
            obj.executeTrajectoryToAbsPose(absAcqPose(1),absAcqPose(2),absAcqPose(3),vmax,1,0,1);
            
            [found, palletPose] = obj.getPalletPose();  %palletPose is relative
            if found
                obj.goToPickUpPose(palletPose);
                pause(.5);
                obj.goToDropPose(absDropPose);
            end
            
            %laser
            
            
            %pickup
            
        end
        
        function [found, palletPose] = getPalletPose(obj)
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
                   [x1, y1, th1] = obj.irToXy(indices(i), ranges(i));
                   xArr = [xArr x1]; yArr = [yArr y1];
            end
            offset = -0.14; %raise if doing two measurements
            [found, palletPose] = obj.findLineCandidate(ranges, xArr, yArr, offset);
            scatter(xArr, yArr, 'g');
            %obj.robot.stopLaser();
            pause(1);
        end
        
        function [found, palletPose] = exerciseOne(obj)
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
            [found, palletPose] = obj.findLineCandidate(ranges, xArr, yArr, offset);
            scatter(xArr, yArr, 'g');
            %obj.robot.stopLaser();
            pause(1);
        end
        
        
        function [Inertia, centroidX, centroidY, numPoints, distance, Ixx, Ixy, Iyy] = getInertia(obj, pointSetX, pointSetY)
            numPoints = length(pointSetX);
            centroidX = mean(pointSetX); centroidY = mean(pointSetY); 
            distance = sqrt(centroidX^2 + centroidY^2);
            pointSetXCloud = pointSetX - centroidX; pointSetYCloud = pointSetY - centroidY;

            Ixx = sum(pointSetXCloud.^2);
            Iyy = sum(pointSetYCloud.^2);
            Ixy = -1*sum(pointSetXCloud.*pointSetYCloud);
            Inertia = [Ixx Ixy;Ixy Iyy] / numPoints; % normalized
        end
        
        function [found, palletPose] = findLineCandidate(obj, ranges, xArr, yArr, offset)
            scatter(xArr, yArr, 'g');
            centroidXFinal = 0; centroidYFinal = 0; orientationFinal = 0; found = false;
            hold on;
            count = 0;  minDistance = 4.0; maxPoints = 0;
            addlPointX = 0; addlPointY = 0; 
            for i = 1:length(ranges)
                pointSetX = []; pointSetY = [];
                x = xArr(i); y = yArr(i);
                if x < 0
                    continue;
                end
                minDist = 4.0;
                for j = 1:length(ranges)
                    otherX = xArr(j); otherY = yArr(j);
                    dist = sqrt((otherX - x)^2 + (otherY - y)^2);
                    threshold = 0.01;
                    if dist <= 0.0635 + threshold
                        pointSetX = [pointSetX otherX];
                        pointSetY = [pointSetY otherY];
                    else
                        if dist < minDist
                            addlPointX = otherX;
                            addlPointY = otherY;
                            minDist = dist;
                        end
                    end
                    
                end
                newPointSetX = [pointSetX addlPointX];
                newPointSetY = [pointSetY addlPointY];
                
                [Inertia2, centroidX2, centroidY2, numPoints2, dist2, ixx, ixy, iyy] = obj.getInertia(newPointSetX, newPointSetY);
                
                [Inertia, centroidX, centroidY, numPoints, distance, Ixx, Ixy, Iyy] = obj.getInertia(pointSetX, pointSetY);
                
                eig2 = eig(Inertia2);
                
                lambda2 = eig(Inertia);
                lambda = eig(Inertia);
                
                if (eig2(2) - lambda(2) < .00025)
                    continue;
                end
                
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
                        orientation = atan2(2*Ixy,Iyy-Ixx)/2.0;
                        centroidXFinal = centroidX; centroidYFinal = centroidY; 
                        orientationFinal = orientation;
                        
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
                palletPose = [centroidXFinal, centroidYFinal, orientation];
            else
                palletPose = [0, 0, 0];  %THIS MIGHT BE WRONGGGGGG!!!!
            end
            fprintf("count %d\n", count);
        end
        
        
        
        
        function executeTrajectoryToAbsPose(obj,xfa,yfa,thfa,vmax,sgn,useMap, iteration)      
            % Execute a trajectory using state estimator as feedback      
            % Plan the trajectory. The terminal pose is specifie      
            % in world coordinates (not start relative). 
            poseG = [xfa,yfa,thfa];
            poseR = [obj.sensedX, obj.sensedY, obj.sensedTh]; %obj.startPose;
            referencePose = obj.matToPoseVec(inv(obj.bToA(poseR))*obj.bToA(poseG));
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(referencePose(1), ... %-xfa+obj.startPose(1), ...
                referencePose(2), ... %yfa-obj.startPose(2)
                referencePose(3), sgn); %thfa-obj.startPose(3), sgn);
            obj.trajectoryObj.planVelocities(vmax);%0.2);
            obj.executeTrajectory(iteration);
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
        
        function relPose = absToRel(obj, absPose) %NOT pose object, but array
            poseR = [obj.sensedX, obj.sensedY,obj.sensedTh];
            relPose = obj.matToPoseVec(obj.aToB(poseR)*obj.bToA(absPose));
        end
       
        function absPose = relToAbs(obj, relPose)
            poseR = [obj.sensedX, obj.sensedY, obj.sensedTh];
            absPose = test.matToPoseVec(test.bToA(poseR)*test.bToA(relPose));
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
                %figure(100 + iteration);
                parms = trajectory.getParms();
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
                %poseEst = pose(obj.startPose(1), obj.startPose(2), obj.startPose(3));
                poseEst = pose(obj.sensedX, obj.sensedY, obj.sensedTh);
                fprintf("poseest %d %d %d \n", x(poseEst), y(poseEst), th(poseEst));
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
                    
                    if (preval2 ~= currval2)
                        newRangeImage = true;
                        poseEst = obj.updateStateFromEncodersAtTime(newxEnc,newyEnc,tcurr, bodyPts, false);
                        preval2 = currval2;
                    else
                       poseEst = obj.updateStateFromEncodersAtTime(newxEnc,newyEnc,tcurr, bodyPts, true);
                       newRangeImage = false;
                    end
                    
                    xEnc = newxEnc; yEnc = newyEnc;
                    sensedXArr = [sensedXArr x(poseEst)];
                    sensedYArr = [sensedYArr y(poseEst)];
                   
                    
                    if ((tcurr >= tDelay) && (tcurr <= lastT + tDelay))
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
                        kth = 1/tao; %1/0.2;%tao;
                        rpr = (mat^-1)*[errorx; errory];
                        thekx = 1/tao;
                    
                        %prevV = trajectory.getVAtTime(tcurr-tDelay);
                        theky = 2/(prevV*tao^2);
                        if (prevV < 0.05)
                            theky = 0;
                        end
                        up = [thekx*rpr(1); theky*rpr(2) + kth*errorth];

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
                    
                    
                    [vl, vr]  = robotModel.VwTovlvr(V, w);
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
                tcurr = toc(tStart);
                poseEst =obj.updateStateFromEncodersAtTime(newxEnc, newyEnc, tcurr, bodyPts, false);
                sensedXArr = [sensedXArr x(poseEst)];
                sensedYArr = [sensedYArr y(poseEst)];
                obj.sensedX = x(poseEst);
                obj.sensedY = y(poseEst);
                obj.sensedTh = th(poseEst);
                
%                 referenceXArr = [referenceXArr finalPose(1)];
%                 referenceYArr = [referenceYArr finalPose(2)];
                obj.robot.sendVelocity(0,0);
                
                fprintf("1 - %d %d \n", referenceYArr(end), x(poseEst));
                fprintf("2 - %d %d \n", referenceXArr(end), y(poseEst));
                sqrt((referenceYArr(end) - x(poseEst))^2 + (referenceXArr(end) - y(poseEst))^2)
                %pause(2);
%                 if iteration ~= 1
%                     hold on;
%                 end
                %figure(10 + iteration);
                %plot(obj.tarr, referenceXArr, obj.tarr, referenceYArr, obj.tarr, referenceThArr);
                %legend("x", "y", "th");


                figure(100);
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
%                 figure(iteration + 23);
%                 plot(obj.tarr, obj.errorxarr, obj.tarr, obj.erroryarr, obj.tarr, obj.errortharr);
%                 xlabel('time');
%                 ylabel('error');
%                 legend("x", "y", "th");
%                 title("Trajectory" + iteration);
%                 
                
                figure(63);
                plot(obj.lidarX, obj.lidarY, obj.odometryX, obj.odometryY);
                xlabel('x');
                ylabel('y');
                legend("lidar", "odometry");
                title("Lidar vs Odometry " + iteration);
%                 
                
%                 figure(13 + iteration);
%                 plot(obj.tarr, referenceThArr, obj.tarr,sensedThArr); 
%                 title("THETAAA");
%                 xlabel('Position x (m)');
%                 ylabel('Position y (m)');
%                 %fprintf("error %d \n", sqrt((obj.sensedX-referencePose(1))^2 + (obj.sensedY-referencePose(2))^2));
%                 legend("reference", "sensed");
%                 hold on;
                
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