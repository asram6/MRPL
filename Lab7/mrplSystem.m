classdef mrplSystem
    
    properties
            controllerObj,
            robot,
            trajectoryObj,
            estRobot, 
            sensedX,
            sensedY, theta,
            startPose
    end
    
    methods
        
        

        function obj = mrplSystem()
            %obj.controllerObj = controller();
            obj.robot = raspbot('Raspbot-26');
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
            %fprintf("in executeTrajtoRelativePose 1 \n");
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            %fprintf("in executeTrajtoRelativePose 2 \n");
            obj.trajectoryObj.planVelocities(0.2);
            %fprintf("in executeTrajtoRelativePose 3 \n");
            obj.executeTrajectory(iteration);
            %fprintf("in executeTrajtoRelativePose 4 \n");
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
                parms = trajectory.getParms();
                fprintf("in executeTrajectory before loop \n");
                %lastI = size(trajectory.timeArray);
                lastT = trajectory.getTrajectoryDuration();%timeArray(lastI(2));
                tao = .4; %0.7;
                prevV = 0;
                error = 0;
                errory = 0;
                errorth = 0;
                tDelay = 0.11;
                tcurr = 0;
                %obj.estRobot.x = 0;  obj.estRobot.y = 0; 
                obj.estRobot.prevEncoderY = yEnc; %obj.estRobot.theta = 0;
                obj.estRobot.prevEncoderX = xEnc;
                obj.estRobot.tPrev = 0;
                while (s <= parms(3) && (tcurr < lastT + tDelay)) % || (errorth) > 0.02)
                    if (firstLoop) 
                        firstLoop = false;
                        tStart = tic;
                        tcurr = 0;
                    end
                    while (preval == currval)
                       pause(0.001);
                    end
                    preval = currval;
                    newxEnc = obj.robot.encoders.LatestMessage.Vector.X;
                    newyEnc = obj.robot.encoders.LatestMessage.Vector.Y;
                    prevT = tcurr;
                    tcurr = toc(tStart);
                    obj.estRobot.integrate(newxEnc, newyEnc, tcurr);
                    
                    obj.sensedX = obj.estRobot.x;
                    obj.sensedY = obj.estRobot.y;
                    obj.theta = obj.estRobot.theta;
                    ds = ((newxEnc - xEnc)+(newyEnc-yEnc))/2;
                    s = s + ds;
                    %t = trajectory.getTimeAtDist(s) + 0.005;
                    %fprintf("got time at dist   t = %d\n", t);
                    xEnc = newxEnc; yEnc = newyEnc;
                    sensedXArr = [sensedXArr obj.sensedX];
                    sensedYArr = [sensedYArr obj.sensedY];
                    if (tcurr-tDelay > lastT)
                        referencePose = finalPose;
                        matrix = obj.bToA(obj.startPose)*obj.bToA(referencePose);
                        referencePose = obj.matToPoseVec(matrix);
                    else
                        referencePose = trajectory.getPoseAtTime(tcurr-tDelay);
                        matrix = obj.bToA(obj.startPose)*obj.bToA(referencePose);
                        referencePose = obj.matToPoseVec(matrix);
                    end
                    errorx = referencePose(1) - obj.sensedX; errory = referencePose(2)-obj.sensedY;
                    errorth = referencePose(3) - obj.theta;
                    error = sqrt(errorx^2 + errory^2);
                    mat = zeros(2,2);%3,3);
                    mat(1,1) = cos(obj.theta); mat(1,2) = -sin(obj.theta); %mat(1,3) = x;
                    mat(2,1) = sin(obj.theta); mat(2,2) = cos(obj.theta); %mat(2,3) = y;
                    %mat(3,1) = 0.0; mat(3,2) = 0.0; mat(3, 3) = 1.0;
                    kth = 1/0.3;%tao;
                    rpr = (mat^-1)*[errorx; errory];
                    thekx = 1/tao;
                    prevV = trajectory.getVAtTime(tcurr-tDelay);
                    theky = 2/(prevV*tao^2);
                    if (prevV < 0.05)
                        theky = 0;
                    end
                    up = [thekx*rpr(1); theky*rpr(2) + kth*errorth];
                    prevW = trajectory.getwAtTime(tcurr-tDelay);
                    V = prevV + up(1);
                    w = prevW + up(2);
                    referenceXArr = [referenceXArr referencePose(1)];
                    referenceYArr = [referenceYArr referencePose(2)];
                    %fprintf('V = %d w = %d\n', V, w);
                    [vl, vr]  = robotModel.VwTovlvr(V, w);
                    %fprintf('%d %d\n', vl,vr);
                    [vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
                    
                    obj.robot.sendVelocity(vl, vr);
                    pause(0.08);
                    
                end
                %obj.estRobot.integrate(newxEnc, newyEnc, tcurr);
                obj.robot.sendVelocity(0,0);
                pause(2);
                figure(iteration);
                plot(referenceXArr, referenceYArr, sensedXArr, sensedYArr); 
                title("Reference Trajectory versus the Sensed Trajectory");
                xlabel('Position x (m)');
                ylabel('Position y (m)');
                fprintf("error %d \n", sqrt((obj.sensedX-referencePose(1))^2 + (obj.sensedY-referencePose(2))^2));
                legend("reference", "sensed");
        end
    end
        
end


function encoderEventListener(handle, event)
    global currval;
    currval = ~currval;  
end