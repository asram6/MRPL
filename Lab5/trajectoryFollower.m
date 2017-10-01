classdef trajectoryFollower
    properties
        controllerObj
    end
    
    methods
        function obj = trajectoryFollower(vArr, wArr)
            obj.controllerObj = controller();
            obj.feedForward(vArr, wArr);
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
    end
    
end