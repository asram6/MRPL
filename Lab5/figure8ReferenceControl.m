classdef figure8ReferenceControl
    
    properties
        tPause
        v
        sf
        tf
        ktheta
        kk
        Tf
        ks
        kv
        T 
        trajectory
        totalTime
    end
    
    methods(Static = true)
        function obj = figure8ReferenceControl(Ks,Kv,tPause)
         % Construct a figure 8 trajectory. It will not start until
         % tPause has elapsed and it will stay at zero for tPause
         % afterwards. Kv scales velocity up when > 1 and Ks scales
         % the size of the curve itself up.
         obj.v = 0.2;
         obj.sf = 1;
         obj.tf = obj.sf/obj.v;
         obj.ktheta = 2*pi/obj.sf;
         obj.kk = 15.1084;
         obj.Tf = Ks/Kv*obj.tf;
         obj.tPause = tPause;
         obj.ks = Ks;
         obj.kv = Kv;
        end
        
        function obj = oldFigure8ReferenceControl(Ks,Kv,tPause)
         % Construct a figure 8 trajectory. It will not start until
         % tPause has elapsed and it will stay at zero for tPause
         % afterwards. Kv scales velocity up when > 1 and Ks scales
         % the size of the curve itself up.
         obj.v = 0.2;
         obj.sf = 1;
         obj.tf = obj.sf/obj.v;
         obj.ktheta = 2*pi/obj.sf;
         obj.kk = 15.1084;
         obj.Tf = Ks/Kv*obj.tf;
         obj.tPause = tPause;
         obj.ks = Ks;
         obj.kv = Kv;
         %robot = raspbot('Raspbot-26');
         %pause(3);
         %robot.sendVelocity(0,0);
         %pause(3);
         T = 0;
         obj.T = T;
         totalTime = figure8ReferenceControl.getTrajectoryDuration(obj);
         obj.totalTime = totalTime;
         firstLoop = true;
         tdelay = 0.11;
         while (T < totalTime)
             if (firstLoop)
                 %tStart = tic;
                 T = 0;
             end
             t = (Kv/Ks)*(T-tdelay);
             [V w] = figure8ReferenceControl.computeControl(obj, t);
             [vl vr] = robotModel.VwTovlvr(V, w);
             pause(0.01);
             T = toc(tStart);
             obj.T = [obj.T T];
         end
         %robot.shutdown();
        end


        function [V w] = computeControl(obj,timeNow)
         % Return the linear and angular velocity that the robot 
         % should be executing at time timeNow. Any zero velocity
         % pauses specified in the constructor are implemented here
         % too.
         totalTime = figure8ReferenceControl.getTrajectoryDuration(obj);
         if ((timeNow < obj.tPause) || (totalTime - timeNow < obj.tPause) || (timeNow > totalTime))
            V = 0; w = 0;
         else 
             st = obj.v*timeNow;
             k = (obj.kk/obj.ks)*sin(obj.ktheta*st);
             V = obj.kv*obj.v;
             w = k*V;
         end

        end

         function duration = getTrajectoryDuration(obj)
         % Return the total time required for motion and for the
         % initial and terminal pauses.
         duration = 2*obj.tPause + obj.Tf;

         end 
    end
end