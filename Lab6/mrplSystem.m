classdef mrplSystem
    
    properties
            controllerObj,
            robot,
            trajectoryObj
    end
    
    methods
        
        

        function obj = mrplSystem()
            obj.controllerObj = controller();
            obj.robot = raspbot('Raspbot-26');
            pause(4);
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            pause(3);
            obj.executeTrajectories();
            pause(.1);
        end
            
        function executeTrajectoryToRelativePose(obj, x, y, th, sgn)
            obj.trajectoryObj = cubicSpiralTrajectory.planTrajectory(x, y, th, sgn);
            obj.trajectoryObj.planVelocities(0.2);
            obj.executeTrajectory();
        end
        
        
        
        function executeTrajectories(obj)
            xf1 = 0.3048; yf1 = 0.3048; thf1 = 0.0;
            obj.executeTrajectoryToRelativePose(xf1, yf1, thf1, 1);
            pause(2);
            %obj.mrplSystem.executeTrajectoryToRelativePose(cf1, yf1, thf1, -1);

            xf2 = -0.6096; yf2 = -0.6096; thf2 = -pi()/2.0;
            obj.executeTrajectoryToRelativePose(xf2, yf2, thf2, 1);
            pause(2);
            %obj.mrplSystem.executeTrajectoryToRelativePose(cf1, yf1, thf1, -1);
            
            xf3 = -0.3048; yf3 = 0.3048; thf3 = pi()/2.0; 
            obj.executeTrajectoryToRelativePose(xf3, yf3, thf3, 1);
            pause(2);
            %obj.mrplSystem.executeTrajectoryToRelativePose(cf1, yf1, thf1, -1);
        end

        function executeTrajectory(obj)
                global currval;
                global preval;
                preval = false; currval = false;
                trajectory = obj.trajectoryObj;
                firstLoop = true;
                tStart = 0;
                tcurr = 0;
                x = obj.robot.encoders.LatestMessage.Vector.X;
                y = obj.robot.encoders.LatestMessage.Vector.Y;
                s = 0;
                
                parms = trajectory.getParms();
                while (s <= parms(3))
                    while (preval == currval)
                       pause(0.001);
                    end
                    preval = currval;
                    %fprintf("out of loop\n");
                    newx = obj.robot.encoders.LatestMessage.Vector.X;
                    newy = obj.robot.encoders.LatestMessage.Vector.Y;
                    ds = ((newx - x)+(newy-y))/2;
                    s = s + ds;
                    t = trajectory.getTimeAtDist(s) + 0.01;
                    %fprintf("got time at dist   t = %d\n", t);
                    x = newx; y = newy;
                    V = trajectory.getVAtTime(t);
                    w = trajectory.getwAtTime(t);
                    %fprintf('V = %d w = %d\n', V, w);
                    [vl, vr]  = robotModel.VwTovlvr(V, w);
                    %fprintf('%d %d\n', vl,vr);
                    [vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
                    
                    obj.robot.sendVelocity(vl, vr);
                    pause(0.05);
                end
                obj.robot.sendVelocity(0,0);
        end
    end
        
end


function encoderEventListener(handle, event)
    global currval;
    currval = ~currval;  
end