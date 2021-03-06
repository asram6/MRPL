classdef mrplSystemfeedforward
    
    properties
            controllerObj,
            robot,
            trajectoryObj,
            estRobot, 
            sensedX,
            sensedY, theta,
            startPose,
            errorxarr, erroryarr, errortharr, varr, warr, tarr
    end
    
    methods
        
        

        function obj = mrplSystemfeedforward()
            %obj.controllerObj = controller();
            obj.robot = raspbot('Raspbot-30');
            pause(4);
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            pause(3);
            obj.sensedX = 0; obj.sensedY = 0; obj.theta = 0;
            encoderX = obj.robot.encoders.LatestMessage.Vector.X; 
            encoderY = obj.robot.encoders.LatestMessage.Vector.Y;
            obj.estRobot = simRobot1(encoderX, encoderY);

            obj.executeTrajectories();
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
            xf1 = 0.3048; yf1 = 0.3048; thf1 = 0.0;
            %xf1 = 0; yf1 = 0.9144; thf1 = pi;
            obj.executeTrajectoryToRelativePose(xf1, yf1, thf1, 1, 1);
            obj.startPose = [xf1; yf1; thf1];
            pause(2);
            
            xf2 = -0.6096; yf2 = -0.6096; thf2 = -pi()/2.0;
            obj.executeTrajectoryToRelativePose(xf2, yf2, thf2, 1, 2);
            pause(2);
            obj.startPose = [xf2; yf2; thf2] + obj.startPose;
            
            xf3 = -0.3048; yf3 = 0.3048; thf3 = pi()/2.0; 
            obj.executeTrajectoryToRelativePose(xf3, yf3, thf3, 1, 3);
            pause(2);
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
                s = 0;
                referenceXArr = []; referenceYArr = []; sensedXArr = []; sensedYArr = [];
                finalPose = trajectory.getFinalPose();
                figure(iteration); 
                %figure(iteration + 20); 
                figure(iteration + 5);
                figure(100);
                parms = trajectory.getParms();
                fprintf("in executeTrajectory before loop \n");
                %lastI = size(trajectory.timeArray);
                lastT = trajectory.getTrajectoryDuration();%timeArray(lastI(2));
                tao = 0.5;%.27; %0.7;
                prevV = 0;
                error = 0;
                errory = 0;
                errorth = 0;
                tDelay = 0.137;%0.11;
                tcurr = 0;
                obj.estRobot.prevEncoderY = yEnc; %obj.estRobot.theta = 0;
                obj.estRobot.prevEncoderX = xEnc;
                obj.estRobot.tPrev = 0;
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
                    obj.tarr = [obj.tarr tcurr];
                    
                    obj.estRobot.integrate(newxEnc, newyEnc, tcurr, encoderEventTime);
                    
                    obj.sensedX = obj.estRobot.x;
                    obj.sensedY = obj.estRobot.y;
                    obj.theta = obj.estRobot.theta;
                    ds = ((newxEnc - xEnc)+(newyEnc-yEnc))/2;
                    s = s + ds;

                    xEnc = newxEnc; yEnc = newyEnc;
                    sensedXArr = [sensedXArr obj.sensedX];
                    sensedYArr = [sensedYArr obj.sensedY];
                    if (tcurr-tDelay > lastT)
                        referencePose = finalPose;
                        matrix = obj.bToA(obj.startPose)*obj.bToA(referencePose);
                        referencePose = obj.matToPoseVec(matrix);
                        prevV = trajectory.getVAtTime(lastT);
                        prevW = trajectory.getwAtTime(lastT);
                    elseif (tcurr > tDelay)
                        referencePose = trajectory.getPoseAtTime(tcurr-tDelay);
                        matrix = obj.bToA(obj.startPose)*obj.bToA(referencePose);
                        referencePose = obj.matToPoseVec(matrix);
                        prevV = trajectory.getVAtTime(tcurr-tDelay);
                        prevW = trajectory.getwAtTime(tcurr-tDelay);
                    else
                        referencePose = trajectory.getPoseAtTime(0);
                        matrix = obj.bToA(obj.startPose)*obj.bToA(referencePose);
                        referencePose = obj.matToPoseVec(matrix);
                        prevV = trajectory.getVAtTime(0);
                        prevW = trajectory.getwAtTime(0);
                    
                    end
                    errorx = referencePose(1) - obj.sensedX; errory = referencePose(2)-obj.sensedY;
                    errorth = referencePose(3) - obj.theta;
                    errorth = atan2(sin(errorth),cos(errorth));
                    obj.errorxarr = [obj.errorxarr errorx];
                    obj.erroryarr = [obj.erroryarr errory];
                    obj.errortharr = [obj.errortharr errorth];
                    error = sqrt(errorx^2 + errory^2);


                    V = prevV; 
                    obj.varr = [obj.varr V];
                    w = prevW; 
                    w = atan2(sin(w),cos(w));
                    obj.warr = [obj.warr w];
                    referenceXArr = [referenceXArr referencePose(1)];
                    referenceYArr = [referenceYArr referencePose(2)];
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
                obj.estRobot.integrate(newxEnc, newyEnc, tcurr, encoderEventTime);
                obj.robot.sendVelocity(0,0);
                pause(2);
%                 if iteration ~= 1
%                     hold on;
%                 end 
                figure(100);
                plot(referenceXArr, referenceYArr, sensedXArr, sensedYArr); 
                title("Reference Trajectory versus the Sensed Trajectory");
                xlabel('Position x (m)');
                ylabel('Position y (m)');
                fprintf("error %d \n", sqrt((obj.sensedX-referencePose(1))^2 + (obj.sensedY-referencePose(2))^2));
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
                
                figure(iteration + 5);
                plot(obj.tarr, obj.varr, obj.tarr, obj.warr);
                xlabel('time');
                ylabel('velocity/ omega');
                legend("v", "w");
                title("Trajectory" + iteration);
%                 %plot(obj.warr, obj.errorxarr, obj.warr, obj.erroryarr, obj.warr, obj.errortharr);
        end
    end
        
end


function encoderEventListener(handle, event)
    global currval;
    currval = ~currval;  
end