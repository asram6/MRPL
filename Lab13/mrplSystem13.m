classdef mrplSystem13 < handle
    properties
        localizer; robot; estRobot; startPose; trajectoryObj;
        errorxarr; erroryarr ; errortharr; varr; warr; tarr; 
        odometryX;odometryY; odometryTh; lidarX; lidarY; sensedX; sensedY; sensedTh;
        dropPoses; vmax; pickPose; pickingUp; vmaxFirst; onFirst; dropPose; lidarTh;
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
                    %thOffset = 0.07;
                    thOffset = 0.03;%0.087;%0.087;%0.03;
                    th = (i-1)*(pi/180)-thOffset;
                    if (th > pi)
                        th = th-2*pi;
                    end
                    x = r*cos(th);
                    y = r*sin(th);
        end
        
        function obj = mrplSystem14()
            obj.robot = raspbot('Raspbot-24');
            pause(2);
            while (true)
                droppedOff = obj.checkDroppedOff();
               % fprintf("droppedOff %d \n", droppedOff);
                pause(2);
            end
           % fprintf("in here\n");
            %obj.testRangeImage();
        end
            
        function obj = mrplSystem13()
            obj.vmaxFirst = 0.07;%0.1;
            obj.onFirst = true;
            
            %obj.odometryX = [];obj.odometryY = []; obj.lidarX = []; obj.lidarY = [];
            p1 = [0 ; 0];
            p2 = [ 0 ; 1.2192];
            p3 = [ 1.2192 ;  0];
            p4 = [1.2192; 1.2192];
            lines_p1 = [p1 p1 p3];
            lines_p2 = [p2 p3 p4];
            
            p5 = [0;0];
            p6 = p2*2;
            p7 = p3*2;
            p8 = p4*2;
            
            lines13a = [p5,p5,p7];
            lines13b = [p6 p7 p8];
            obj.pickingUp = true;

            obj.vmax = 0.07;%0.07;
            
            obj.localizer = lineMapLocalizer13(lines_p1, lines_p2, 0.3, 0.01, 0.0005);
            %obj.localizer = lineMapLocalizer13(lines13a, lines13b, 0.3, 0.01, 0.0005);
            
            obj.robot = raspbot('Raspbot-24');
            pause(2);
            
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            obj.robot.laser.NewMessageFcn = @laserEventListener;
            
            pause(2);
            encoderX = obj.robot.encoders.LatestMessage.Vector.X;
            encoderY = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.estRobot = simRobot1(encoderX, encoderY);
            obj.robot.startLaser();
            pause(2);
            obj.robot.forksDown();
            pause(0.5);
            obj.lab13ChallengeTask(); 
        end
        
        function poseEst = updateStateFromEncodersAtTime(obj,newx,newy,tcurr, bodyPts, flag)
            %fprintf("odometry before update x %d, y %d, th %d\n", obj.estRobot.x, obj.estRobot.y, obj.estRobot.theta);
            if (tcurr ~= 0)
                obj.estRobot.integrate(newx, newy, tcurr);
            end
            newX = obj.estRobot.x; newY = obj.estRobot.y; newTh = obj.estRobot.theta;
            %fprintf("odometry after update x %d, y %d, th %d\n", newX, newY, newTh);
            %step 1: odometry pose
           
            poseEst = pose(newX, newY, newTh);
            obj.odometryX = newX;
            obj.odometryY = newY;
            
                       
            if (~flag)
                %step 2: lidar pose 
                robotBodyPts = poseEst.bToA()*bodyPts;
                pts = obj.robot.laser.LatestMessage.Ranges;
                xArr = []; yArr = []; thArr = []; wArr = [];
                tStart = tic();
                tcurr = toc(tStart);
                for i = 1:length(pts)
                    if (mod(i, 8) == 0) %used to be 8
                        [x,y,th] = obj.irToXy(i, pts(i));
                        %if ((th <= pi/6) && (th >= -pi/6))
                            xArr = [xArr x];
                            yArr = [yArr y];
                            wArr = [wArr 1.0];
                        %end
                    end
                end
                tcurr = toc(tStart);
                %fprintf("boop\n");
                pointsInModelFrame = [xArr ; yArr; wArr];
                
                ids = obj.localizer.throwOutliers(poseEst, pointsInModelFrame);

                allIds = linspace(1, length(pointsInModelFrame), length(pointsInModelFrame));
                
                goodIds = setdiff(allIds, ids);
                pointsInModelFrame = pointsInModelFrame(:, goodIds);

                %fprintf("beep...");
                
                [~, poseLidar] = obj.localizer.refinePose(poseEst, pointsInModelFrame, 10, robotBodyPts);
                %fprintf("boop\n");
                obj.lidarX = poseLidar.x(); obj.lidarY = poseLidar.y(); obj.lidarTh = poseLidar.th();
                %step 3: new pest from pest and plid
                k = 0.25;%0.25; % <= 1/4
                newTh = newTh + k*(obj.lidarTh - newTh);
                newTh = atan2(sin(newTh),cos(newTh));
                newX = newX + k*(obj.lidarX-newX);
                newY = newY + k*(obj.lidarY-newY);
                poseEst = pose(newX,newY,newTh);
            end
            obj.sensedX = newX; obj.sensedY = newY; obj.sensedTh = newTh;
        end
            
       
        function updateSensedState(obj, tcurr, flag)
            global preval; global currval;
            bodyPts = robotModel.bodyGraph();   
            while (preval == currval)
                       pause(0.001);
            end
            preval = currval;
            newxEnc = obj.robot.encoders.LatestMessage.Vector.X;
            newyEnc = obj.robot.encoders.LatestMessage.Vector.Y;
           % tStart = tic;
           % tcurr = toc(tStart);
            obj.updateStateFromEncodersAtTime(newxEnc, newyEnc, tcurr, bodyPts, flag);
        end
            
        function moveRelDist(obj,dist, doControlplotting)
            % move forward or backward a specified distance and stop.
            % make sure the velocity is such that the distance will take at
            % least a second
            sgn = 1;
            if dist < 0
                sgn = -1;
            end
            
            obj.executeTrajectoryToRelativePose(abs(dist), 0, 0, sgn, 1345);
            %obj.startPose = [abs(dist); 0; 0];
        end
        
        function turnRelAngle(obj, angle, doControlplotting, flag)
            % make sure the velocity is such that the distance will take at
            % least a secondFill me in
            % move a distance forward or backward
            global preval; global currval;
            seconds = 1.5; %3.0;
            %angle = angle/2;
            w = angle/seconds;
            V = 0;
            tstart = tic;
            tcurr = toc(tstart);
            %tcurr = toc(tstart);
            %obj.estRobot.tPrev = tcurr;
           % fprintf("before turn before update x %d, y %d, th %d\n", obj.sensedX, obj.sensedY, obj.sensedTh);
            while (toc(tstart) < seconds)
                tcurr = toc(tstart);
                obj.updateSensedState(tcurr, flag);
                [vl, vr]  = robotModel.VwTovlvr(V, w);
                [vl, vr] = robotModel.limitWheelVelocities([vl, vr]); 
                obj.robot.sendVelocity(vl, vr);
                pause(0.5);
                
            end
            obj.robot.sendVelocity(0, 0);
            pause(0.1);
            tcurr = toc(tstart);
           % fprintf("after turn before update x %d, y %d, th %d\n", obj.sensedX, obj.sensedY, obj.sensedTh);
            obj.estRobot.tPrev = 0;
%             while (preval == currval)
%                    pause(0.001);
%             end
%             preval = currval;
%             xEnc = obj.robot.encoders.LatestMessage.Vector.X;
%             yEnc = obj.robot.encoders.LatestMessage.Vector.Y;
%             obj.estRobot.prevEncoderX = xEnc;
%             obj.estRobot.prevEncoderY = yEnc;
            
            
            obj.updateSensedState(tcurr, flag);
           % fprintf("after turn after update x %d, y %d, th %d\n", obj.sensedX, obj.sensedY, obj.sensedTh);
            
        end
        
        function executeTrajectoryToRelativePose(obj, x, y, th, sgn, iteration)
            %fprintf("x %d   y %d   th %d \n", x, y, th);
            obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            obj.trajectoryObj.planVelocities(obj.vmax);
            obj.executeTrajectory2(iteration);
        end
        
        function checkForPallet(obj, absPalletPose)
            %FOR LAB 13
        end
        
        %Lab 12, exercise 2, for labs 12 and 13
        function [closestPose, poses] = getClosestPose(obj, poses)
            dimensions = size(poses);
            length = dimensions(1);
            closestPose = poses(1,:);
            index = 1;
            minDist = sqrt((closestPose(1)-obj.sensedX)^2+(closestPose(2)-obj.sensedY)^2);
            for i=2:length
                pose = poses(i,:);
                dist = sqrt((pose(1)-obj.sensedX)^2+(pose(2)-obj.sensedY)^2);
                if (dist < minDist)
                    closestPose = pose;
                    index = i;
                    minDist = dist;
                end
            end
           fprintf("dropping off at x %d, y %d, th %d, legthDropPoses %d\n", closestPose(1), closestPose(2), closestPose(3), length)
           % fprintf("closestPose %d %d, minDist %d, currPose %d %d\n", closestPose(1),closestPose(2),minDist,obj.sensedX, obj.sensedY);
            [poses, ~] = removerows(poses,index);
        end
        
        function lab13ChallengeTask(obj)
%             pickPoses = [];
%             pickX = 0;
%             pickY = 1.8288;
%             pickTh = pi/2;
%             for i=1:7
%                 pickX = pickX + 0.3048;
%                 pickPoses = [pickPoses; [pickX,pickY,pickTh]];
%             end
%             pickX = 2.1336;
%             pickTh = 0;
%             pickY = 0.6096;
%             for i=1:3
%                 pickPoses = [pickPoses; [pickX, pickY, pickTh]];
%                 pickY = pickY + 0.3048;
%             end
%             obj.dropPoses = [];
%             dropX = 0.4572;
%             dropY = 0.3048;
%             dropTh = -pi/2;
%             for i = 1:7
%                 obj.dropPoses = [obj.dropPoses; [dropX, dropY, dropTh]];
%                 dropX = dropX + 0.1524;
%                 end
%             pickPoses
%             obj.dropPoses
            obj.startPose = [0.2286; 0.2286; pi()/2];
            obj.sensedX = obj.startPose(1); obj.sensedY = obj.startPose(2); obj.sensedTh = obj.startPose(3);
            %obj.estRobot.x = 0.2286; obj.estRobot.y = 0.2286; obj.estRobot.theta = pi()/2; not needed anymore (done in simRobot constructor) 
            
            %This is here so that we can switch to the smaller case to test
            pickPoses = [[0.3048, 1.0668, pi/2]; [0.6098, 1.0688, pi/2];[0.9144, 1.0688, pi/2]];  %0.9144
            obj.dropPoses = [[0.5344, .1524, -pi/2]; [0.6858, 0.1524, -pi/2];[0.8382, 0.1524, -pi/2]];
            
            while (~isempty(pickPoses)) 
                [obj.pickPose, pickPoses] = obj.getClosestPose(pickPoses);                
                obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
                %fprintf("pick up %d %d %d \n", pickPose(1),pickPose(2),pickPose(3));
                obj.pickDropObject(obj.pickPose);
            end
        end
        
        %TODO: use trajectory here instead, change search area to small
        %wedge, increase offset in pickdropObj
        function pickUpPallet(obj, relDestPose)
         
            offset = -0.2;%-0.06;
            Tro = relDestPose;
            Tog = [offset, 0, 0];
            pickUpPose = obj.matToPoseVec(obj.bToA(Tro)*obj.bToA(Tog));
            %obj.executeTrajectoryToRelativePose(pickUpPose);
            absPick = obj.relToAbs(pickUpPose);
            
            
            %SECOND MOTION
            
            brokenOut = obj.executeTrajectoryToAbsPose(absPick(1), absPick(2), absPick(3), obj.vmax, 1, 1, 1);
            obj.estRobot.tPrev = 0;
            pause(0.2);
            %%%check for broken out
            
            
            if (~brokenOut) 
                %THIRD MOTION go straight
                pause(0.5);
                %obj.moveRelDist(0.14, 0);
                %pause(0.1);
               % fprintf("ABOUT TO GO STRAIGHT\n");
                obj.robot.sendVelocity(0.1,0.1);
                %obj.moveRelDist(0.05, 0);
                pause(1.5);
                obj.robot.sendVelocity(0,0);
                pause(0.5);
                obj.updateSensedState(0, false);
                pause(0.5);
                obj.robot.forksUp();
                pause(0.5);
                obj.pickingUp = false;
            end
            
        end
        
        function dropOffPallet(obj, absDestPose)
            offset = 0.0;%-0.12;
            relDestPose = obj.absToRel(absDestPose);
            %Tro = relDestPose;
            %Tog = [offset, 0, 0];
            %dropPose = obj.matToPoseVec(obj.bToA(Tro)*obj.bToA(Tog));
            absDropPose = absDestPose;%obj.relToAbs(dropPose);
            
            obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
            
            angle = atan2(relDestPose(2), relDestPose(1));
            obj.turnRelAngle(angle, 0, true);
                        
            obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
            relDestPose = obj.absToRel(absDestPose);
            
            %fprintf("rel drop pose %d %d %d \n", relDestPose(1), relDestPose(2), relDestPose(3));
            fprintf("abs drop pose %d %d %d \n", absDropPose(1), absDropPose(2), absDropPose(3));
            obj.executeTrajectoryToAbsPose(absDropPose(1),absDropPose(2),absDropPose(3),obj.vmaxFirst, 1, 1, 1);
            obj.robot.forksDown();
            pause(0.5);
            obj.moveRelDist(-0.07, 1);
            pause(0.1);
            droppedOff = obj.checkDroppedOff();
            if (~droppedOff)
                droppedOff = obj.checkDroppedOff();
            end
            if (~droppedOff)
                fprintf("IN NOT DROPPED OFF\n");
                obj.dropPoses = [obj.dropPoses; obj.dropPose];
            end
        end
        
        function pickDropObject(obj, absPickPose)
            Tro = obj.absToRel(absPickPose); %Tro = relPickPose
            %fprintf("abs pick pose %d %d %d \n", absPickPose(1), absPickPose(2), absPickPose(3));
            %fprintf("tro pose %d %d %d \n", Tro(1), Tro(2), Tro(3));
           
            offset = -0.45; 
            %end
            Tog = [offset, 0, 0];
            
            acqPose = obj.matToPoseVec(obj.bToA(Tro)*obj.bToA(Tog)); %acquisition pose, can offset more
            absAcqPose = obj.relToAbs(acqPose);
            %fprintf("acq pose %d %d %d \n", acqPose(1), acqPose(2), acqPose(3));
            
            %currPose = [obj.sensedX, obj.sensedY, obj.sensedTh];
            turnAngle = atan2(acqPose(2), acqPose(1));
            %fprintf("ACQ POSE x %d, y %d, th %d,   turnangle %d\n", acqPose(1), acqPose(2), acqPose(3), turnAngle);
            %turnAngle = acqPose(3);
            turnThreshold = pi/16;
            if (turnAngle > turnThreshold) %can change angle threshold
                obj.turnRelAngle(turnAngle, 0, false); %test to see if negative
            end
            
            %fprintf("current pose %d %d %d \n", obj.sensedX, obj.sensedY, obj.sensedTh);
            %fprintf("abs acq pose %d %d %d \n", absAcqPose(1), absAcqPose(2), absAcqPose(3));
            
            %FIRST MOTION
            obj.pickingUp = true;
           % obj.onFirst = true;
            brokenOut = obj.executeTrajectoryToAbsPose(absAcqPose(1),absAcqPose(2),absAcqPose(3),obj.vmaxFirst,1,0,1);
           % obj.onFirst = false;
            %check if we broke out of this trajectory
            
            
            pause(0.5);
            obj.updateSensedState(0, false);
            pause(0.5);
            if (~brokenOut)
                relPickPose = obj.absToRel(absPickPose);
                [found, palletPose] = obj.getPalletPose(relPickPose);  %palletPose is relative
                %fprintf("found:%d, pose: %d %d %d \n", found, palletPose(1), palletPose(2), palletPose(3));
                if found
                    obj.pickUpPallet(palletPose);
                    %pause(.5);
            
                    [absDropPose, obj.dropPoses] = obj.getClosestPose(obj.dropPoses);
                    obj.dropPose = absDropPose;
                    %fprintf("drop off %d %d %d \n", absDropPose(1),absDropPose(2),absDropPose(3));
                    obj.dropOffPallet(absDropPose);
                end
            end
        end
        
        function [found, palletPose] = getPalletPose(obj, relPickPose)
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
            [found, palletPose] = obj.findLineCandidate(ranges, xArr, yArr, offset, relPickPose);
            %scatter(xArr, yArr, 'g');
            %obj.robot.stopLaser();
            %pause(1);
        end
        
        function testRangeImage(obj)
            obj.robot.startLaser();
            pause(2);
            while (true)
                pts = obj.robot.laser.LatestMessage.Ranges;
                xArr = []; yArr = []; thArr = []; wArr = [];
                for i = 1:length(pts)
                    %if (mod(i, 2) == 0) %used to be 8
                        [x,y,th] = obj.irToXy(i, pts(i));
                        %if ((th <= pi/6) && (th >= -pi/6))
                            xArr = [xArr x];
                            yArr = [yArr y];
                            wArr = [wArr 1.0];
                        %end
                    %end
                end
                figure(1);
                scatter(xArr, yArr);
                pause(0.5);
            end
        
        end
        
        function [Inertia, centroidX, centroidY, numPointsLab13, distanceLab13, Ixx, Ixy, Iyy] = getInertiaLab13(obj, pointSetX, pointSetY)
            numPointsLab13 = length(pointSetX);
            centroidX = mean(pointSetX); centroidY = mean(pointSetY); 
            distanceLab13 = sqrt(centroidX^2 + centroidY^2);
            pointSetXCloud = pointSetX - centroidX; pointSetYCloud = pointSetY - centroidY;

            Ixx = sum(pointSetXCloud.^2);
            Iyy = sum(pointSetYCloud.^2);
            Ixy = -1*sum(pointSetXCloud.*pointSetYCloud);
            Inertia = [Ixx Ixy;Ixy Iyy] / numPointsLab13; % normalized
        end
        
        function [found, palletPose] = findLineCandidate(obj, ranges, xArr, yArr, offset, relPickPose)
           % fprintf("TOP OF FIND LINE CANDIDATE\n");
            %scatter(xArr, yArr, 'g');
            centroidXFinal = 0; centroidYFinal = 0; orientationFinal = 0; found = false;
            %hold on;
            count = 0;  minDistance = 4.0; maxPoints = 0;
            addlPointX = 0; addlPointY = 0; 
           % figure(27);
           % scatter(-relPickPose(2),relPickPose(1), 'r');
           % hold on;
            for i = 1:length(ranges)
                pointSetX = []; pointSetY = [];
                x = xArr(i); y = yArr(i);
                if x < 0
                    continue;
                end
                %if ((th > pi/4) || (th < -pi/4))
                %    continue;
                %end
                %fprintf("skipped th %d\n", th);
                %distFromPickPose = sqrt(((x-0.02)-relPickPose(1))^2 + (y-relPickPose(2))^2);
                %scatter(-y,x, 'b');
                %hold on;
                %if (distFromPickPose > 0.2)%~(y < 0.1 && y > -0.1)   %this is trying to only get wedge in front of us (and not wall)
                    %fprintf("in find line candidate %d \n", y);
                    %fprintf("IN IF dist %d     x %d  y %d     relpickpose x %d   relpickpose y %d   \n", distFromPickPose, x, y, relPickPose(1), relPickPose(2));
                    
                %    continue;
                %else
                    %fprintf("dist %d\n", distFromPickPose);
                    %fprintf("else dist %d     x %d  y %d     relpickpose x %d   relpickpose y %d   \n", distFromPickPose, x, y, relPickPose(1), relPickPose(2));
               % end
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
                
                [Inertia2, centroidX2, centroidY2, numPoints2, dist2, ixx, ixy, iyy] = obj.getInertiaLab13(newPointSetX, newPointSetY);
                
                [Inertia, centroidX, centroidY, numPoints, distance, Ixx, Ixy, Iyy] = obj.getInertiaLab13(pointSetX, pointSetY);
                
                eig2 = eig(Inertia2);
                
                lambda2 = eig(Inertia);
                lambda = eig(Inertia);
                
                if (eig2(2) - lambda(2) < 0.0009)%.00009)%45)
                    %fprintf("eig: %d\n", eig2(2) - lambda(2));
                    continue;
                end
                
                diff = eig2(2) - lambda(2);
                %fprintf("DIFF %d \n", diff);
                lambda = sqrt(lambda)*1000.0;
                distanceThreshold = 0.01;
                numPoints = length(pointSetX);
                %distance = tmp;
                %fprintf("numPoints %d, distance %d length %d\n",numPoints, distance, length(pointSetX));
                if ((numPoints >= 7) && (lambda(1) < 1.3) && (numPoints >= maxPoints) && (distance < minDistance + distanceThreshold)) %%THIS USED TO BE 7
                    leftX = min(pointSetX); topY = max(pointSetY);
                    rightX = max(pointSetX); bottomY = min(pointSetY);
                    diagonal = sqrt((rightX - leftX)^2 + (bottomY - topY)^2);
                    %fprintf("line 396!!!!\n");
                    if abs(diagonal - 0.127) <= 0.035
                        
                       
                        %fprintf("find  line candidate = %d \n", y);
                        found = true;
                        maxPoints = numPoints;
                        minDistance = distance;
                        count = count + 1;
                        if (count == 11)
                            count = 0;
                        end
                        orientation = atan2(2*Ixy,Iyy-Ixx)/2.0;
                        centroidXFinal = centroidX; centroidYFinal = centroidY; 
                        figure(7);
                        if (((orientation < 0)))% && (centroidX > 0)) || ((orientation >= 0) && (centroidX <= 0)))
                           
                            plot([leftX, rightX], [bottomY, topY]);
                        else
                            plot([leftX, rightX], [topY, bottomY]);
                        end
                        hold on;
                        plot(centroidX, centroidY, 'r*');
                        %fprintf("centroidx %d, centroidy %d, orientation %d \n", centroidX, centroidY, orientation);
                        palletPose = [centroidXFinal, centroidYFinal, orientation];
                        absPose = obj.relToAbs(palletPose);
                        theDist = sqrt((absPose(1) - obj.pickPose(1))^2 + (absPose(2) - obj.pickPose(2))^2);
                        
                        %once we figure out how close these poses are when
                        %it's looking at the correct one, when this dist is
                        %too big we know that the pallet we were looking
                        %for is not actually there
                        %So, found should be false if the distance is not
                        %small enough
                       % fprintf("SEE IF ANY OF THESE HAVE A SMALL DISTANCE AND HOW SMALL: pose found: x %d y %d th %d    pickup pose x %d y %d th %d    distance between: %d\n", absPose(1), absPose(2), absPose(3), ...
                         %   obj.pickPose(1), obj.pickPose(2), obj.pickPose(3), theDist);
                        %fprintf("centroidXFinal %d, centroidYFinal %d, orientation %d \n", centroidXFinal, centroidYFinal, orientation);
                        %fprintf("ABS x %d, y %d, th %d FOUND %d\n", absPose(1), absPose(2), absPose(3), found);
                        hold on;
                    end
                    
                end
                
            end
            
                        
            if (found)
                
                palletPose = [centroidXFinal, centroidYFinal, orientation];
            else
                palletPose = [0, 0, 0];  %THIS MIGHT BE WRONGGGGGG!!!!
            end
            absPose = obj.relToAbs(palletPose);
            %fprintf("centroidXFinal %d, centroidYFinal %d, orientation %d \n", centroidXFinal, centroidYFinal, orientation);
            %fprintf("ABS x %d, y %d, th %d FOUND %d\n", absPose(1), absPose(2), absPose(3), found);
            %fprintf("count %d\n", count);
           % fprintf("END OF FIND LINE\n");
        end
         
        
        function brokenOut = executeTrajectoryToAbsPose(obj,xfa,yfa,thfa,vmax,sgn,useMap, iteration)      
            % Execute a trajectory using state estimator as feedback      
            % Plan the trajectory. The terminal pose is specifie      
            % in world coordinates (not start relative). 
            poseG = [xfa,yfa,thfa];
            poseR = [obj.sensedX, obj.sensedY, obj.sensedTh]; %obj.startPose;
            obj.startPose = [obj.sensedX; obj.sensedY; obj.sensedTh];
            referencePose = obj.matToPoseVec(inv(obj.bToA(poseR))*obj.bToA(poseG));
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(referencePose(1), ... %-xfa+obj.startPose(1), ...
                referencePose(2), ... %yfa-obj.startPose(2)
                referencePose(3), sgn); %thfa-obj.startPose(3), sgn);
            obj.trajectoryObj.planVelocities(vmax);%0.2);
            brokenOut = obj.executeTrajectory2(iteration);
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
            absPose = obj.matToPoseVec(obj.bToA(poseR)*obj.bToA(relPose));
        end
        
    
        
        function droppedOff = checkDroppedOff(obj)
            pts = obj.robot.laser.LatestMessage.Ranges;
            xArr = []; yArr = [];
            droppedOff = false;
            for i = 1:length(pts)
                [x,y,th] = obj.irToXy(i, pts(i));
                if ((th <= pi/6) && (th >= -pi/6))
                    xArr = [xArr x];
                    yArr = [yArr y];
                    if (x < 0.2)
                        % fprintf("x %d y %d\n", x, y);
                    end
                    if ((x > 0.06) && (x < 0.3048) && (abs(y) <0.02))
                        droppedOff = true;
                       % fprintf("dropped off!\n");
                        break;
                    end
                    
                end 
            end
            
            
        end
    
        function broke = executeTrajectory2(obj, iteration)
               % fprintf("TOP OF EXECUTE TRAJ\n");
                obj.estRobot.tPrev = 0; 
                global currval; global currval2;
                global preval; global preval2;
                referenceXArr = []; referenceYArr = []; sensedXArr = []; sensedYArr = [];sensedThArr = [];referenceThArr = [];
                preval = false; currval = false;
                preval2 = false; currval2 = false;
                trajectory = obj.trajectoryObj;
                firstLoop = true;
                while (preval == currval)
                       pause(0.001);
                end
                preval = currval;
                xEnc = obj.robot.encoders.LatestMessage.Vector.X;
                yEnc = obj.robot.encoders.LatestMessage.Vector.Y;
                
                lastT = trajectory.getTrajectoryDuration();
                
                tao = 2.5; %2.5;%0.6;%0.4;%.27; %0.7;
                taoX = 2.5;%3.5;%3.25; %2.5; 
                taoTh = 2.5;%1.5;%2.5;%2.0;
                taoY = 2.5;%1.5;%2.0;%2.5;
                finalPose = trajectory.getFinalPose();
              %  fprintf("Ref final pose x %d, y %d, th %d; Sensed x %d, y %d, th %d\n", finalPose(1), finalPose(2), finalPose(3), ...
               %     obj.sensedX, obj.sensedY, obj.sensedTh);
                tDelay = 0.115;%0.11;
                tcurr = 0;
                obj.estRobot.prevEncoderY = yEnc; %obj.estRobot.theta = 0;
                obj.estRobot.prevEncoderX = xEnc;
                broke = false;
                poseEst = pose(obj.sensedX, obj.sensedY, obj.sensedTh);
                count = 0;
                bodyPts = robotModel.bodyGraph();
                while (tcurr < lastT + tDelay) 
                    count = count + 1;
                    if (firstLoop) 
                        firstLoop = false;
                        tStart = tic;
                    end
                    %fprintf("before while encoders in first while\n");
                    while (preval == currval)
                       pause(0.001);
                    end
                    %fprintf("after while encoders in first while\n");
                    %fprintf("tcurr %d, lastT %d, tDelay %d\n", tcurr, lastT, tDelay);
                    preval = currval;
                    newxEnc = obj.robot.encoders.LatestMessage.Vector.X;
                    newyEnc = obj.robot.encoders.LatestMessage.Vector.Y;
                    
                    tcurr = toc(tStart);
                    if (tcurr < tDelay)
                        V = 0;
                        w = 0;
                    elseif (tcurr > lastT)
                        V = 0;
                        w = 0;
                    end
                    
                    if (preval2 ~= currval2)
                        if ~obj.pickingUp
                            pickUpFlag = true;
                        else
                            pickUpFlag = false;
                        end
                        poseEst = obj.updateStateFromEncodersAtTime(newxEnc,newyEnc,tcurr, bodyPts, pickUpFlag);
                        preval2 = currval2;
                    else
                        poseEst = obj.updateStateFromEncodersAtTime(newxEnc,newyEnc,tcurr, bodyPts, true);
                    end
                    
%                     if ((count == 10) && (obj.pickingUp))
%                         distFromPallet = sqrt((poseEst.x()-obj.pickPose(1))^2+(poseEst.y()-obj.pickPose(2))^2);
%                         relPickPose = obj.absToRel(obj.pickPose);
%                         if (distFromPallet < 1.2)
%                             [found, ~] = obj.getPalletPose(relPickPose);
%                             if (~found)
%                                 obj.robot.sendVelocity(0,0);
%                                 pause(0.11);
%                                 fprintf("BROKEN OUT\n");
%                                 obj.pickingUp = false;
%                                 broke = true;
%                                 break;
%                             end
%                         end
%                     end
                    
                    sensedXArr = [sensedXArr x(poseEst)];
                    sensedYArr = [sensedYArr y(poseEst)];
                    
                    
                    if ((tcurr >= tDelay) && (tcurr <= lastT + tDelay))
                        referencePose = trajectory.getPoseAtTime(tcurr-tDelay);
                        
                        matrix = obj.bToA(obj.startPose)*obj.bToA(referencePose);
                        referencePose = obj.matToPoseVec(matrix);
                        
                        prevV = trajectory.getVAtTime(tcurr-tDelay);
                        prevW = trajectory.getwAtTime(tcurr-tDelay);

                        errorx = referencePose(1) - poseEst.x(); 
                        errory = referencePose(2)-y(poseEst);
                        
                        refTh = referencePose(3);
                        refTh = atan2(sin(refTh),cos(refTh));
                        sensedTheta = th(poseEst);
                        sensedTheta = atan2(sin(sensedTheta), cos(sensedTheta));
                        errorth =  refTh - sensedTheta;
                        errorth = atan2(sin(errorth),cos(errorth));
                        
                        mat = zeros(2,2);
                        mat(1,1) = cos(sensedTheta); mat(1,2) = -sin(sensedTheta);
                        mat(2,1) = sin(sensedTheta); mat(2,2) = cos(sensedTheta); 
                        kth = 1/taoTh; 
                        rpr = (mat^-1)*[errorx; errory];
                        thekx = 1/taoX;
                    
                        theky = 2/(prevV*taoY^2);
                        if (prevV < 0.05)
                            theky = 0;
                        end
                        up = [thekx*rpr(1); theky*rpr(2) + kth*errorth];

                        if (abs(errorth) < pi()/600 || abs(errory) < 0.0002)
                            up = [0;0];
                        end
                        V = prevV + up(1);
                    
                        up2 = up(2);
                        up2 = atan2(sin(up2), cos(up2));
                        
                        w = prevW + up2;
                        w = atan2(sin(w),cos(w));
                        referenceXArr = [referenceXArr referencePose(1)];
                        referenceYArr = [referenceYArr referencePose(2)];
                        referenceThArr = [referenceThArr referencePose(3)];
                        
                    end
                    
                    
                    [vl, vr]  = robotModel.VwTovlvr(V, w);
                    [vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
                    
                    obj.robot.sendVelocity(vl, vr);
                    pause(0.01);
                    
                    
                end
               % fprintf("BEFORE WHILE ENCODER\n");
                if (~broke)
                    while (preval == currval)
                       pause(0.001);
                    end
                    %fprintf("AFTER WHILE ENCODER\n");

                    preval = currval;
                    newxEnc = obj.robot.encoders.LatestMessage.Vector.X;
                    newyEnc = obj.robot.encoders.LatestMessage.Vector.Y;
                    tcurr = toc(tStart);
                    if (preval2 ~= currval2)
                        if ~obj.pickingUp
                            pickUpFlag = true;
                        else
                            pickUpFlag = false;
                        end
                        poseEst = obj.updateStateFromEncodersAtTime(newxEnc,newyEnc,tcurr, bodyPts, pickUpFlag);
                        preval2 = currval2;
                    else
                        poseEst = obj.updateStateFromEncodersAtTime(newxEnc,newyEnc,tcurr, bodyPts, true);
                    end

                    obj.sensedX = x(poseEst);
                    obj.sensedY = y(poseEst);
                    obj.sensedTh = th(poseEst);
                    obj.robot.sendVelocity(0,0);
                    if (iteration ~= 1345)
                        figure(1345);
                        plot(referenceXArr, referenceYArr, sensedXArr, sensedYArr); 
                        title("Reference Trajectory versus the Sensed Trajectory");
                        xlabel('Position x (m)');
                        ylabel('Position y (m)');
                        legend("reference", "sensed");
                        hold on;
                    end
                end
                %if (~obj.pickingUp)
               % fprintf("Ref final pose x %d, y %d, th %d; Sensed x %d, y %d, th %d; last in ref array x %d, y %d \n", finalPose(1), finalPose(2), finalPose(3), ...
                    %obj.sensedX, obj.sensedY, obj.sensedTh, referenceXArr(end), referenceYArr(end));
                %end
               % fprintf("END OF EXECUTE TRAJ\n");
               obj.estRobot.tPrev = 0; 
                
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