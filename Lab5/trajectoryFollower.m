classdef trajectoryFollower
    properties
        controllerObj,
        robotTrajObj,
        robot
    end
    
    methods
        function obj = trajectoryFollower()
            obj.controllerObj = controller();
            obj.robot = raspbot('Raspbot-26');
           % obj.feedForward(vArr, wArr);
            obj.robotTrajObj = robotTrajectory(figure8ReferenceControl(1,1,.01));
            obj.feedBack();
        end
        
        function feedForward(obj, vArr, wArr)
            vlarr = [];
            vrarr = [];
            len = size(vArr);
            for i = 1:len(2)
                V = vArr(i);
                w = wArr(i);
                [vl1 vr1] = robotModel.VwTovlvr(V, w);
                [vl vr] = robotModel.limitWheelVelocities([vl1 vr1]);
                vlarr = [vlarr vl];
                vrarr = [vrarr vr];    
            end
            
            obj.controllerObj.sendVelocities(vlarr, vrarr);
        end
        
        function feedBack(obj)
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            preval = false; currval = false;
            tStart = tic;
            t = toc(tStart);
            x = obj.robot.encoders.LatestMessage.Vector.X;
            y = obj.robot.encoders.LatestMessage.Vector.Y;
            
            th = 0;
            Tf = 10;
            while (t < Tf)
                while (preval == currval)
                   pause(0.001);
                end
                fprintf("got out \n");
                preval = currval;
                T = toc(tStart);
                [x, y, th] = obj.updatePose(x, y, th, t, T);
                refPose = obj.robotTrajObj.getPoseAtTime(t);
                error = refPose - [x; y; th];
                pose2 = [x;y];
                mat = zeros(2,2);
                mat(1,1) = cos(th);
                mat(1,2) = -sin(th);
                mat(2,1) = sin(th);
                mat(2,2) = cos(th);
                matinv = inv(mat);
                errorConverted = matinv*pose2;
                obj.controllerObj.doOneIteration(errorConverted, obj.robot);
                t = toc(tStart);
            end
        end
        
        function [x, y, th] = updatePose(obj, prevx, prevy, prevth, t, T)
            encoderX = obj.robot.encoders.LatestMessage.Vector.X;
            encoderY = obj.robot.encoders.LatestMessage.Vector.Y;
            diffX = encoderX - prevx;
            diffY = encoderY - prevy;
            dt = (T - t);
            vlactual = diffX/dt;
            vractual = diffY/dt;
            Vactual = (vlactual+vractual)/2;
            Wactual = (vractual - vlactual)/0.088;
            th = prevth + Wactual*dt;
            x = prevx + Vactual*cos(th)*dt;
            y = prevy + Vactual*sin(th)*dt;
        end
        
        function encoderEventListener(handle, event)
            fprintf("in event listener\n");
            global currval;
            currval = ~currval;  
        end
        
    end
end