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
            fprintf("in executeTrajtoRelativePose 1 \n");
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            fprintf("in executeTrajtoRelativePose 2 \n");
            obj.trajectoryObj.planVelocities(0.2);
            fprintf("in executeTrajtoRelativePose 3 \n");
            obj.executeTrajectory(iteration);
            fprintf("in executeTrajtoRelativePose 4 \n");
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
                tStart = tic;
                tcurr = 0;
                x = obj.robot.encoders.LatestMessage.Vector.X;
                y = obj.robot.encoders.LatestMessage.Vector.Y;
                s = 0;
                theta = 0; sensedX = 0; sensedY = 0;
                referenceXArr = []; referenceYArr = []; sensedXArr = []; sensedYArr = [];
                figure(iteration);
                parms = trajectory.getParms();
                fprintf("in executeTrajectory before loop \n");
                while (s <= parms(3))
                    while (preval == currval)
                       pause(0.001);
                    end
                    preval = currval;
                    %fprintf("out of loop\n");
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
                    t = trajectory.getTimeAtDist(s) + 0.01;
                    %fprintf("got time at dist   t = %d\n", t);
                    x = newx; y = newy;
                    sensedXArr = [sensedXArr sensedX];
                    sensedYArr = [sensedYArr sensedY];
                    V = trajectory.getVAtTime(t);
                    w = trajectory.getwAtTime(t);
                    referencePose = trajectory.getPoseAtTime(t);
                    referenceXArr = [referenceXArr referencePose(1)];
                    referenceYArr = [referenceYArr referencePose(2)];
                    %fprintf('V = %d w = %d\n', V, w);
                    [vl, vr]  = robotModel.VwTovlvr(V, w);
                    %fprintf('%d %d\n', vl,vr);
                    [vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
                    
                    obj.robot.sendVelocity(vl, vr);
                    pause(0.05);
                end
                fprintf("after loop \n");
                figure(iteration);
                fprintf("after iteration thing \n");
                plot(referenceXArr, referenceYArr, sensedXArr, sensedYArr); 
                legend("reference", "sensed");
                fprintf("iteration %d \n", iteration);
                obj.robot.sendVelocity(0,0);
        end
    end
        
end


function encoderEventListener(handle, event)
    global currval;
    currval = ~currval;  
end