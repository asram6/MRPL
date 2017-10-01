classdef trajectoryFollower
    properties
        controllerObj,
        robotTrajObj,
        robot,
        trajectoryObj
    end
    
    methods
        function obj = trajectoryFollower()
            obj.controllerObj = controller();
            obj.robot = raspbot('Raspbot-26');
           % obj.feedForward(vArr, wArr);
            obj.robotTrajObj = robotTrajectory(figure8ReferenceControl(1,1,.01));
            obj.trajectoryObj = obj.robotTrajObj;
            %obj.feedBack();

            pause(.1);
        end
        
        function feedForward(obj)
            trajectory = obj.trajectoryObj;
            firstLoop = true;
            tStart = 0;
            tcurr = 0;
            while (tcurr < 6)
                if (firstLoop) 
                    firstLoop = false;
                    tStart = tic;
                    tcurr = toc(tStart);
                end
                tcurr = toc(tStart);
                V = robotTrajectory.getVelocityAtTime(trajectory, tcurr);
                w = robotTrajectory.getOmegaAtTime(trajectory, tcurr);
                [vl, vr]  = robotModel.VwTovlvr(V, w);
                %fprintf('%d %d\n', vl,vr);
                %[vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
                
                obj.controllerObj.sendVelocity(vl, vr, obj.robot);
            end
            
            obj.controllerObj.shutdown(robot);
        end
        
        function feedBack(obj)
            obj.robot.encoders.NewMessageFcn = @encoderEventListener;
            global currval;
            global preval;
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
        
        
        
    end
end

function encoderEventListener(handle, event)
            fprintf("in event listener\n");
            global currval;
            currval = ~currval;  
end