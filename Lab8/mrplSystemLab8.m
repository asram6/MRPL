classdef mrplSystemLab8
    
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
        
        
        function executeTrajectoryLab8(obj, thepose)
            obj.startPose = [0;0;0];
            obj.executeTrajectoryToRelativePose(thepose(1), thepose(2), thepose(3), 1, 1);
        end
        
        function obj = mrplSystemLab8(robot)
            %obj.controllerObj = controller();
            obj.robot = robot;
            pause(2);
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            pause(2);
            obj.sensedX = 0; obj.sensedY = 0; obj.sensedTheta = 0;
            encoderX = obj.robot.encoders.LatestMessage.Vector.X; 
            encoderY = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.estRobot = simRobot1(encoderX, encoderY);

            pause(.1);
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
        
        function executeTrajectories(obj)
            obj.startPose = [0;0;0];
            xf1 = 0.9144; yf1 = 0; thf1 = 0;
            %xf1 = 0.3048; yf1 = 0.3048; thf1 = 0.0;
            obj.executeTrajectoryToRelativePose(xf1, yf1, thf1, -1, 1);
%             obj.startPose = [xf1; yf1; thf1];
%             pause(2);
%             xf2 = -0.6096; yf2 = -0.6096; thf2 = -pi()/2.0;
%             obj.executeTrajectoryToRelativePose(xf2, yf2, thf2, 1, 2);
%             pause(2);
%             obj.startPose = [xf2; yf2; thf2] + obj.startPose;
%             
%             xf3 = -0.3048; yf3 = 0.3048; thf3 = pi()/2.0; 
%             obj.executeTrajectoryToRelativePose(xf3, yf3, thf3, 1, 3);
%             pause(2);
        end

        function executeTrajectory(obj, iteration)
                global currval;
                global preval;
                obj.errorxarr = []; obj.erroryarr = []; obj.errortharr = []; obj.varr = []; obj.warr = []; obj.tarr = [];
                preval = false; currval = false;
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
                figure(iteration); 
                %figure(iteration + 20); 
                %figure(iteration + 5);
                figure(100);
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
                pause(1);
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
                    
                    
                    
                    obj.estRobot.integrate(newxEnc, newyEnc, tcurr, encoderEventTime);
                    
                    obj.sensedX = obj.estRobot.x;
                    obj.sensedY = obj.estRobot.y;
                    obj.sensedTheta = obj.estRobot.theta;
                    ds = ((newxEnc - xEnc)+(newyEnc-yEnc))/2;
                    s = s + ds;
                    %t = trajectory.getTimeAtDist(s) + 0.005;
                    %fprintf("got time at dist   t = %d\n", t);
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
                        
                        %refTh = atan2(sin(refTh), cos(refTh));
                        refTh1 = mod(refTh,(2*pi));
                        sensedTh = mod(obj.sensedTheta, (2*pi));
                        errorth =  refTh1 - sensedTh;
                        errorth = atan2(sin(errorth),cos(errorth));
                        
                        obj.errorxarr = [obj.errorxarr errorx];
                        obj.erroryarr = [obj.erroryarr errory];
                        obj.errortharr = [obj.errortharr errorth];
                        error = sqrt(errorx^2 + errory^2);
                        obj.tarr = [obj.tarr tcurr];
                        mat = zeros(2,2);%3,3);
                        mat(1,1) = cos(obj.sensedTheta); mat(1,2) = -sin(obj.sensedTheta); %mat(1,3) = x;
                        mat(2,1) = sin(obj.sensedTheta); mat(2,2) = cos(obj.sensedTheta); %mat(2,3) = y;
                        %mat(3,1) = 0.0; mat(3,2) = 0.0; mat(3, 3) = 1.0;
                        kth = 1/tao;
                        rpr = (mat^-1)*[errorx; errory];
                        thekx = 1/tao;
                    
                        %prevV = trajectory.getVAtTime(tcurr-tDelay);
                        theky = 2/(prevV*tao^2);
                        if (prevV < 0.05)
                            theky = 0;
                        end
                        up = [thekx*rpr(1); theky*rpr(2) + kth*errorth];
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
                obj.estRobot.integrate(newxEnc, newyEnc, tcurr, encoderEventTime);
                obj.robot.sendVelocity(0,0);
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
                figure(iteration);
                plot(obj.tarr, obj.errorxarr, obj.tarr, obj.erroryarr, obj.tarr, obj.errortharr);
                xlabel('time');
                ylabel('error');
                legend("x", "y", "th");
                title("Trajectory" + iteration);
                
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


function encoderEventListener(handle, event)
    global currval;
    currval = ~currval;  
end