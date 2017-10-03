classdef controller
    
    properties
        robot
    end
    
    methods
        function obj = controller()
            %obj.robot = raspbot('Raspbot-26');
            pause(0.1);
        end
        
        function sendVelocity(obj, vl, vr, robot)
            robot.sendVelocity(vl, vr);
            pause(0.01);
        end
        
        function doOneIteration(obj, error, robot, V, w, flag)
            if (~flag)
                [vl, vr]  = robotModel.VwTovlvr(V, w);
                if ((isnan(vl)) || (isnan(vr)))
                    robot.sendVelocity(0, 0);
                else
                    robot.sendVelocity(vl, vr);
                end
                pause(0.01);
            
            else
                errorx = error(1);
                errory = error(2);
                tao = 10;
                V = 0.05;
                kx = 1/tao;
                uv = kx*errorx;
                ky = 2/((tao^2)*uv);
                uw = ky*errory;
                uv = uv + V;
                uw = uw + w;
                if (abs(uw) > pi/4)
                    if (uw > 0)
                        uw = pi/4;
                    else
                        uw = -pi/4;
                    end
                end
                [vl1, vr1] = robotModel.VwTovlvr(uv, uw);
                [vl, vr] = robotModel.limitWheelVelocities([vl1, vr1]);
                robot.sendVelocity(vl, vr);
                pause(0.01);
            end
            
            
        end
        
        function shutdown(obj, robot)
            robot.sendVelocity(0,0);
            pause(1);
            robot.shutdown();
        end
        
        
    end
    
end