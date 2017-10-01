classdef trajectoryFollower
    properties
        controllerObj
        trajectoryObj
    end
    
    methods
        function obj = trajectoryFollower()
            obj.controllerObj = controller();
            obj.trajectoryObj = robotTrajectory(figure8ReferenceControl(1,1,0.1));
            %obj.feedForward();
            pause(.1);
        end
        
        function feedForward(obj)
            trajectory = obj.trajectoryObj;
            firstLoop = true;
            tStart = 0;
            tcurr = 0;
            while (tcurr < 5)
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
                
                obj.controllerObj.sendVelocity(vl, vr);
            end
            
            obj.controllerObj.shutdown();
        end
    end
    
end