classdef controller
    
    properties
        robot
    end
    
    methods
        function obj = controller()
            obj.robot = raspbot('sim');
        end
        
        function sendVelocities(obj, vlarr, vrarr)
            len = size(vlarr);
            for i = 1:len(2)
                vl = vlarr(i); vr = vrarr(i);
                obj.robot.sendVelocity(vl, vr);
                pause(0.1);
            end
        end
        
       
    end
    
end