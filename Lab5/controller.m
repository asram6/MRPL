classdef controller
    
    properties
        robot
    end
    
    methods
        function obj = controller()
            obj.robot = raspbot('sim');
        end
        
        function sendVelocities(obj, vlarr, vrarr)
            for i = 1:size(vlarr)
                vl = vlarr(i); vr = vrarr(i);
                obj.robot.sendVelocity(vl, vr);
            end
        end
        
       
    end
    
end