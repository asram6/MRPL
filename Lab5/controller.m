classdef controller
    
    properties
        robot
    end
    
    methods
        function obj = controller()
            obj.robot = raspbot('sim');
            pause(0.1);
        end
        
        function sendVelocity(obj, vl, vr)
            obj.robot.sendVelocity(vl, vr);
            pause(0.01);
        end
        
        function shutdown(obj)
            obj.robot.shutdown;
        end
        
        
    end
    
end