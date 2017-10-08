classdef mrplSystem
    
    properties
            controllerObj,
            robot,
            trajectoryObj
    end
    
    methods
        
        

        function obj = mrplSystem()
            %obj.controllerObj = controller();
            obj.robot = raspbot('Raspbot-26');
            pause(4);
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            pause(3);
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
        
        
        
        function executeTrajectories(obj)
            xf1 = 0.3048; yf1 = 0.3048; thf1 = 0.0;
            obj.executeTrajectoryToRelativePose(xf1, yf1, thf1, 1, 1);
            pause(2);
            %obj.mrplSystem.executeTrajectoryToRelativePose(cf1, yf1, thf1, -1);
            fprintf("about to call 2nd time \n");
            xf2 = -0.6096; yf2 = -0.6096; thf2 = -pi()/2.0;
            obj.executeTrajectoryToRelativePose(xf2, yf2, thf2, 1, 2);
            pause(2);
            %obj.mrplSystem.executeTrajectoryToRelativePose(cf2, yf2, thf2, -1);
            fprintf("about to call 3rd time \n");
            xf3 = -0.3048; yf3 = 0.3048; thf3 = pi()/2.0; 
            obj.executeTrajectoryToRelativePose(xf3, yf3, thf3, 1, 3);
            pause(2);
            %obj.mrplSystem.executeTrajectoryToRelativePose(cf3, yf3, thf3, -1);
        end

        function executeTrajectory(obj, iteration)
                global currval;
                global preval;
                preval = false; currval = false;
                trajectory = obj.trajectoryObj;
                firstLoop = true;
                %tStart = tic;
                tcurr = 0;
                x = obj.robot.encoders.LatestMessage.Vector.X;
                y = obj.robot.encoders.LatestMessage.Vector.Y;
                s = 0;
                theta = 0; sensedX = 0; sensedY = 0;
                referenceXArr = []; referenceYArr = []; sensedXArr = []; sensedYArr = [];
                finalPose = trajectory.getFinalPose();
                figure(iteration);
                parms = trajectory.getParms();
                fprintf("in executeTrajectory before loop \n");
                %lastI = size(trajectory.timeArray);
                lastT = trajectory.getTrajectoryDuration();%timeArray(lastI(2));
                tao = .3; %0.7;
                prevV = 0;
                error = 0;
                errory = 0;
                errorth = 0;
                while (s <= parms(3)) % || (error) > 0.02 || (errorth) > 0.02)
                    if (firstLoop) 
                        firstLoop = false;
                        tStart = tic;
                        tcurr = 0;
                    end
                    while (preval == currval)
                       pause(0.001);
                    end
                    preval = currval;
                    newx = obj.robot.encoders.LatestMessage.Vector.X;
                    newy = obj.robot.encoders.LatestMessage.Vector.Y;
                    prevT = tcurr;
                    tcurr = toc(tStart);
                    dt = tcurr- prevT;
                    vlActual = (newx - x)/dt; 
                    vrActual = (newy - y)/dt;
                    vactual = (vlActual + vrActual) /2;
                    omegaActual = (vrActual - vlActual) / 0.088;
                    theta = theta + omegaActual * dt;
                    sensedX = sensedX + vactual*cos(theta)*dt;
                    sensedY = sensedY + vactual*sin(theta)*dt;
                    ds = ((newx - x)+(newy-y))/2;
                    s = s + ds;
                    %t = trajectory.getTimeAtDist(s) + 0.005;
                    %fprintf("got time at dist   t = %d\n", t);
                    x = newx; y = newy;
                    sensedXArr = [sensedXArr sensedX];
                    sensedYArr = [sensedYArr sensedY];
                    if (tcurr > lastT)
                        referencePose = finalPose;
                    else
                        referencePose = trajectory.getPoseAtTime(tcurr);
                    end
                    errorx = referencePose(1) - sensedX; errory = referencePose(2)-sensedY;
                    errorth = referencePose(3) - theta;
                    error = sqrt(errorx^2 + errory^2);
                    mat = zeros(2,2);%3,3);
                    mat(1,1) = cos(theta); mat(1,2) = -sin(theta); %mat(1,3) = x;
                    mat(2,1) = sin(theta); mat(2,2) = cos(theta); %mat(2,3) = y;
                    %mat(3,1) = 0.0; mat(3,2) = 0.0; mat(3, 3) = 1.0;
                    kth = 1/tao;
                    rpr = (mat^-1)*[errorx; errory];
                    thekx = 1/tao;
                    prevV = trajectory.getVAtTime(tcurr);
                    theky = 2/(prevV*tao^2);
                    if (prevV < 0.05)
                        theky = 0;
                    end
                    up = [thekx*rpr(1); theky*rpr(2) + kth*errorth];
                    prevW = trajectory.getwAtTime(tcurr);
                    V = prevV + up(1);
                    w = prevW + up(2);
                    referenceXArr = [referenceXArr referencePose(1)];
                    referenceYArr = [referenceYArr referencePose(2)];
                    %fprintf('V = %d w = %d\n', V, w);
                    [vl, vr]  = robotModel.VwTovlvr(V, w);
                    %fprintf('%d %d\n', vl,vr);
                    [vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
                    
                    obj.robot.sendVelocity(vl, vr);
                    pause(0.01);
                    
                end
                figure(iteration);
                plot(referenceXArr, referenceYArr, sensedXArr, sensedYArr); 
                title("Ref");
                xlabel('Time (seconds)');
                ylabel('Error (m)');
                fprintf("error %d \n", sqrt((sensedX-referencePose(1))^2 + (sensedY-referencePose(2))^2));
                legend("reference", "sensed");
                obj.robot.sendVelocity(0,0);
        end
    end
        
end


function encoderEventListener(handle, event)
    global currval;
    currval = ~currval;  
end