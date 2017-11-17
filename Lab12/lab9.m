classdef lab9 < handle
    
    properties
            controllerObj,
            robot,
            trajectoryObj,
            estRobot, 
            sensedX,
            sensedY, sensedTheta,
            startPose,
            errorxarr, erroryarr, errortharr, varr, warr, tarr
    end
    
    
    
    methods
        function moveRelDist(obj,dist, doControlPlotting)
            % move forward or backward a specified distance and stop.
            % make sure the velocity is such that the distance will take at
            % least a second
            sgn = 1;
            if dist < 0
                sgn = -1;
            end
            obj.executeTrajectoryToRelativePose(abs(dist), 0, 0, sgn, 1);
            obj.startPose = [abs(dist); 0; 0];
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
            while (toc(tstart) < seconds)
                [vl, vr]  = robotModel.VwTovlvr(V, w);
                [vl, vr] = robotModel.limitWheelVelocities([vl, vr]); 
                obj.robot.sendVelocity(vl, vr);
                pause(0.5);
            end
        end
        
        function executeTrajectoryLab8(obj, thepose)
            obj.startPose = [0;0;0];
            obj.sensedX = 0; obj.sensedY = 0; obj.sensedTheta = 0;
            encoderX = obj.robot.encoders.LatestMessage.Vector.X; 
            encoderY = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.estRobot = simRobot1(encoderX, encoderY);
            obj.executeTrajectoryToRelativePose(thepose(1), thepose(2), thepose(3), 1, 1);
        end
        
        function obj = lab9(robot)
            obj.robot = robot;
            pause(2);
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            pause(2);
            obj.sensedX = 0; obj.sensedY = 0; obj.sensedTheta = 0;
            encoderX = obj.robot.encoders.LatestMessage.Vector.X; 
            encoderY = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.estRobot = simRobot1(encoderX, encoderY);
            obj.startPose = [0; 0; 0];
        end
            
        function executeTrajectoryToRelativePose(obj, x, y, th, sgn, iteration)
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            obj.trajectoryObj.planVelocities(0.2);
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
        

        function executeTrajectory(obj, iteration)
                global currval;
                global preval;
                obj.errorxarr = []; obj.erroryarr = []; obj.errortharr = []; obj.varr = []; obj.warr = []; obj.tarr = [];
                preval = false; currval = false;
                trajectory = obj.trajectoryObj;
                firstLoop = true;
                tcurr = 0;
                xEnc = obj.robot.encoders.LatestMessage.Vector.X;
                yEnc = obj.robot.encoders.LatestMessage.Vector.Y;
                startX = xEnc;
                startY = yEnc;
                s = 0;
                referenceXArr = []; referenceYArr = []; sensedXArr = []; sensedYArr = [];
                finalPose = trajectory.getFinalPose();
                parms = trajectory.getParms();
                lastT = trajectory.getTrajectoryDuration();
                tao = 0.4;
                prevV = 0;
                error = 0;
                errory = 0;
                errorth = 0;
                tDelay = 0.115;
                tcurr = 0;
                obj.estRobot.prevEncoderY = yEnc; 
                obj.estRobot.prevEncoderX = xEnc;
                obj.estRobot.tPrev = 0;
                referenceThArr = [];
                pause(1);
                while (tcurr < lastT + tDelay) 
                    if (firstLoop) 
                        firstLoop = false;
                        tStart = tic;
                        tcurr = 0;
                        prevT = 0;
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
                    
                    
                    
                    obj.estRobot.integrate(newxEnc, newyEnc, tcurr);
                    
                    obj.sensedX = obj.estRobot.x;
                    obj.sensedY = obj.estRobot.y;
                    obj.sensedTheta = obj.estRobot.theta;
                    ds = ((newxEnc - xEnc)+(newyEnc-yEnc))/2;
                    s = s + ds;
                    xEnc = newxEnc; yEnc = newyEnc;
                    sensedXArr = [sensedXArr obj.sensedX];
                    sensedYArr = [sensedYArr obj.sensedY];
                    
                    if ((tcurr >= tDelay) && (tcurr <= lastT))
                        referencePose = trajectory.getPoseAtTime(tcurr-tDelay);
                        matrix = obj.bToA(obj.startPose)*obj.bToA(referencePose);
                        referencePose = obj.matToPoseVec(matrix);
                        prevV = trajectory.getVAtTime(tcurr-tDelay);
                        prevW = trajectory.getwAtTime(tcurr-tDelay);

                        errorx = referencePose(1) - obj.sensedX; errory = referencePose(2)-obj.sensedY;
                        refTh = referencePose(3);
                        
                        refTh1 = mod(refTh,(2*pi));
                        sensedTh = mod(obj.sensedTheta, (2*pi));
                        errorth =  refTh1 - sensedTh;
                        errorth = atan2(sin(errorth),cos(errorth));
                        
                        obj.errorxarr = [obj.errorxarr errorx];
                        obj.erroryarr = [obj.erroryarr errory];
                        obj.errortharr = [obj.errortharr errorth];
                        error = sqrt(errorx^2 + errory^2);
                        obj.tarr = [obj.tarr tcurr];
                        mat = zeros(2,2);
                        mat(1,1) = cos(obj.sensedTheta); mat(1,2) = -sin(obj.sensedTheta); 
                        mat(2,1) = sin(obj.sensedTheta); mat(2,2) = cos(obj.sensedTheta);
                        kth = 1/tao;
                        rpr = (mat^-1)*[errorx; errory];
                        thekx = 1/tao;
                    
                        theky = 2/(prevV*tao^2);
                        if (prevV < 0.05)
                            theky = 0;
                        end
                        up = [thekx*rpr(1); theky*rpr(2) + kth*errorth];
                        V = prevV + up(1);
                    
                        up2 = up(2);
                        w = prevW + up2;
                        w = atan2(sin(w),cos(w));
                        referenceXArr = [referenceXArr referencePose(1)];
                        referenceYArr = [referenceYArr referencePose(2)];
                        referenceThArr = [referenceThArr referencePose(3)];
                        obj.varr = [obj.varr V];
                        obj.warr = [obj.warr w];
                    end
                    
                    [vl, vr]  = robotModel.VwTovlvr(V, w);
                    [vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
                    
                    obj.robot.sendVelocity(vl, vr);
                    pause(0.01);
                    
                end
                while (preval == currval)
                   pause(0.001);
                end
                
                encoderEventTime = double(obj.robot.encoders.LatestMessage.Header.Stamp.Sec) + double(obj.robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
                preval = currval;
                newxEnc = obj.robot.encoders.LatestMessage.Vector.X;
                newyEnc = obj.robot.encoders.LatestMessage.Vector.Y;
                tcurr = toc(tStart);
                obj.estRobot.integrate(newxEnc, newyEnc, tcurr);
                obj.robot.sendVelocity(0,0);
        end
    end
        
end


function encoderEventListener(handle, event)
    global currval;
    currval = ~currval;  
end