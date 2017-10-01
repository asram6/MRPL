classdef controller
    
    properties
        robot
    end
    
    methods
        function obj = controller()
            obj.robot = raspbot('Raspbot-26');
        end
        
        function sendVelocities(obj, vlarr, vrarr)
            len = size(vlarr);
            for i = 1:len(2)
                vl = vlarr(i); vr = vrarr(i);
                obj.robot.sendVelocity(vl, vr);
                pause(0.1);
            end
        end
        
        function doOneIteration(obj, error, robot)
            errorx = error(1);
            errory = error(2);
            tao = 150;
            V = 0.05;
            kx = 1/tao;
            ky = 2/((tao^2)*V);
            uv = kx*errorx;
            uw = ky*errory;
            [vl, vr] = robotModel.VwTovlvr(uv, uw);
            fprintf("vl %d, vr %d\n", vl, vr);
            if ((abs(vl) < 0.1) && (abs(vr) < 0.1))
                robot.sendVelocity(vl, vr);
            end
            pause(0.01);
            
            
        end
        
       
    end
    
end