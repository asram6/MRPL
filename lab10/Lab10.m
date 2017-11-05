classdef Lab10
    properties
        localizer; driver;
    end
    
    methods
            
        function obj = Lab10()
            obj.localizer = lineMapLocalizer([0, ], [0, ])
        end
        
    end
end