classdef lineMapLocalizer < handle    
    %mapLocalizer A class to match a range scan against a map in     
    % order to find the true location of the range scan relative to      
    % the map.    
    properties(Constant)        
        maxErr = 0.05; % 5 cm       
        minPts = 5; % min # of points that must match    
    end
    
    properties(Access = private)   
    end
    
    properties(Access = public)        
        lines_p1 = [];     
        lines_p2 = [];    
        gain = 0.0;     
        errThresh = 0.0;     
        gradThresh = 0.0;   
    end
    
    methods
    
        function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)            
            % create a lineMapLocalizer           
            obj.lines_p1 = lines_p1;       
            obj.lines_p2 = lines_p2;     
            obj.gain = gain;         
            obj.errThresh = errThresh;     
            obj.gradThresh = gradThresh;   
        end


        function ro2 = closestSquaredDistanceToLines(obj,pi)            
            % Find the squared shortest distance from pi to any line             
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag

            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));

            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...                    
                    obj.lines_p1(:,i),obj.lines_p2(:,i));               
            end
            ro2 = min(r2Array,[],1);
        end
        


        function ids = throwOutliers(obj,pose,ptsInModelFrame)            
            % Find ids of outliers in a scan.             
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);     
            ids = find(r2 > obj.errThresh*obj.errThresh);        
        end

        function avgErr2 = fitError(obj,pose,ptsInModelFrame)            
            % Find the variance  of perpendicular distances of            
            % all points to all lines
            % transform the points 
            
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);      
            r2(r2 == Inf) = [];     
            err2 = sum(r2);        
            num = length(r2);      
            if(num >= lineMapLocalizer.minPts)          
                avgErr2 = err2/num;       
            else
                % not enough points to make a guess           
                avgErr2 = inf;        
            end
        end


        function [err2_Plus0,J] = getJacobian(obj,poseIn,modelPts)      
            % Computes the gradient of the error function          
            err2_Plus0 = fitError(obj,poseIn,modelPts);      
            eps = 1e-9;
            
            dp_x = [eps ; 0.0 ; 0.0];          
            newPose = pose(poseIn.getPoseVec()+dp_x);
            jacX = fitError(obj, pose(newPose.getPoseVec() - err2_Plus0), modelPts)/eps;
            
            dp_y = [0.0 ; eps ; 0.0];          
            newPose = pose(poseIn.getPoseVec()+dp_y, modelPts);
            jacY = fitError(obj, pose(newPose.getPoseVec() - err2_Plus0), modelPts)/eps;
            
            dp_th = [0.0 ; 0.0; eps];          
            newPose = pose(poseIn.getPoseVec()+dp_th, modelPts);
            jacTh = fitError(obj, pose(newPose.getPoseVec() - err2_Plus0), modelPts)/eps;
            
            J = [jacX, jacY, jacTh];   
            err2_Plus0 = sqrt(err2_Plus0);    
        end
        
        function [err, J] = gradientDescent(obj, pose, modelPts)
            
        end
        
        function [success, outPose] = refinePose(obj, inPose, pointsInModelFrame, maxIters)
            err = intmax;
            grad = intmax;
            errArr = [];
            currIteration = 0;
            ptsThresh = 5;
            outPose = inPose;
            if (length(pointsInModelFrame) < ptsThresh)
                success = false;
            else
                
                while(err >= obj.errThresh && grad >= obj.gradThresh && currIteration < maxIters)
                    [err,J] = obj.getJacobian(inPose, pointsInModelFrame);
                    grad = abs(J(1) + J(2) + J(3));
                    obj.gain = obj.gain - 0.5 * grad; 
                    size(J)
                    a = inPose.x() - obj.gain*J(1);
                    b = inPose.y() - obj.gain*J(2);
                    c = inPose.th() - obj.gain*J(3);
                    
                    inPose = pose(a, b, c);
                    worldBodyPts = inPose.bToA()*pointsInModelFrame;
                    worldBodyPts(1,:)
                    worldBodyPts(2,:)
                    kh = scatter(worldBodyPts(1,:),worldBodyPts(2,:),'k');
                    hold on;
                    
                    currIteration = currIteration + 1;
                   
                end
                currIteration
                if (err < obj.errThresh && grad < obj.gradThresh)
                    success = false;
                else
                    success = true;
                end
                outPose = pose;
            end
            
          
            
        end
        
        function test(obj)
            %set up lines
            p1 = [-2 ; -2];
            p2 = [ 2 ; -2];
            p3 = [ 2 ;  2];
            p4 = [-2 ;  2];
            lines_p1 = [p1 p2 p3 p4];
            lines_p2 = [p2 p3 p4 p1];
            figure(1);
            kh = plot(lines_p1, lines_p2, "r");
            hold on
            
            % Set up test points
            nPts = 10;
            x1   = -2.0*ones(1,nPts);
            x2 = linspace(-2.0,2.0,nPts);
            x3 =  2.0*ones(1,nPts);
            y1 = linspace(0.0,2.0,nPts);
            y2 = 2.0*ones(1,nPts);
            y3 = linspace(2.0,0,nPts);
            w  = ones(1,3*nPts);
            
            x1pts = [x1 x2 x3];
            y1pts = [y1 y2 y3];
            w1pts = w;
            x1pts = [x1pts 100];
            y1pts = [y1pts 100];
            w1pts = [w1pts 100];
            
            pointsInModelFrame = [x1pts ; y1pts ; w1pts];
            
            % pick a pose
            dx = -0.05*rand();
            dy = -0.05*rand();
            dt = -0.05+0.2*rand();
            thePose = pose(0.0+dx,0.0+dy,0.0+dt);
            ids = throwOutliers(obj, pose, pointsInModelFrame);
            
            allIds = linspace(1, length(pointsInModelFrame), length(pointsInModelFrame));
            goodIds = setdiff(allIds, ids);
            pointsInModelFrame = pointsInModelFrame(:, goodIds);
            
            refinePose(obj, thePose, pointsInModelFrame, 10);
            
        end
    
    
    end
end

    
        
    
    
    