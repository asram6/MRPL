classdef robotTrajectory < handle
    properties
        timeArr
        distanceArr
        poseArr
        refControl
        velocityArr
        omegaArr
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
            obj.poseArr = p; obj.timeArr = []; obj.distanceArr = [si]; obj.velocityArr = []; obj.omegaArr = [];
            for i = 1:(numSamples-1)
                ti = (i-1) * dt;
                obj.timeArr = [obj.timeArr ti];
                [vi, wi] = figure8ReferenceControl.computeControl(obj.refControl, ti);
                obj.velocityArr = [obj.velocityArr vi];
                obj.omegaArr = [obj.omegaArr wi];
                si = si + (vi * dt);
                obj.distanceArr = [obj.distanceArr si];
                thnew = mod((p(3) + wi*dt),(2*pi));
                thetaArr = [thetaArr thnew];
                xnew = p(1) + vi*cos(thnew);
                xarr = [xarr xnew];
                ynew = p(2) + vi*sin(thnew);
                yarr = [yarr ynew];
                p = [xnew; ynew; thnew];
                obj.poseArr = [obj.poseArr p];
            end
           obj.timeArr = [obj.timeArr numSamples*dt];
           [vn, wn] = figure8ReferenceControl.computeControl(obj.refControl, numSamples*dt);
           obj.velocityArr = [obj.velocityArr vn];
           obj.omegaArr = [obj.omegaArr wn];
           %trajectoryFollower(velocityArr, omegaArr);
           %plot(xarr, yarr);
        end
        
        function velocity  = getVelocityAtTime(obj,t)
            velocity = interp1(obj.timeArr, transpose(obj.velocityArr), t);
        end
        
        function omega  = getOmegaAtTime(obj,t)
            [times, index] = unique(obj.timeArr);
            omega = interp1(times, obj.omegaArr(index), t);
        end
    end
    
    methods
        
        function pose  = getPoseAtTime(obj,t)
            pose = interp1(obj.timeArr, transpose(obj.poseArr), t);
        end
        
        
        
        
        function distance  = getDistanceAtTime(obj,t)
            distance = interp1(obj.timeArr, transpose(obj.distanceArr), t);
        end
    end
end