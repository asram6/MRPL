classdef robotTrajectory < handle
    properties
        timeArr
        distanceArr
        poseArr
        refControl
    end
    methods(Static = true)
        function obj = robotTrajectory(referControl)
            obj.refControl = referControl;
            robotTrajectory.generateSamples(obj, 0, 5, 1000);
        end
        
        function generateSamples(obj, tstart, tend, numSamples)
           
            p = [0; 0; 0];
            si = 0;
            xarr = []; yarr = []; thetaArr = [0];
            dt = (tend - tstart)/numSamples;
            poseArr = p; timeArr = []; distanceArr = [si]; velocityArr = []; omegaArr = [];
            for i = 1:(numSamples-1)
                ti = (i-1) * dt;
                timeArr = [timeArr ti];
                [vi, wi] = figure8ReferenceControl.computeControl(obj.refControl, ti);
                velocityArr = [velocityArr vi];
                omegaArr = [omegaArr wi];
                si = si + (vi * dt);
                distanceArr = [distanceArr si];
                thnew = mod((p(3) + wi*dt),(2*pi));
                thetaArr = [thetaArr thnew];
                xnew = p(1) + vi*cos(thnew);
                xarr = [xarr xnew];
                ynew = p(2) + vi*sin(thnew);
                yarr = [yarr ynew];
                p = [xnew; ynew; thnew];
                poseArr = [poseArr p];
            end
           timeArr = [timeArr numSamples*dt];
           [vn, wn] = figure8ReferenceControl.computeControl(obj.refControl, numSamples);
           velocityArr = [velocityArr vn];
           omegaArr = [omegaArr wn];
           trajectoryFollower(velocityArr, omegaArr);
           %plot(xarr, yarr);
        end
        
        function pose  = getPoseAtTime(obj,t)
            interp1(obj.timeArr, transpose(obj.poseArr), t);
        end
        
        function velocity  = getVelocityAtTime(obj,t)
            interp1(obj.timeArr, transpose(obj.velocityArr), t);
        end
        
        function distance  = getDistanceAtTime(obj,t)
            interp1(obj.timeArr, transpose(obj.distanceArr), t);
        end
    end
end