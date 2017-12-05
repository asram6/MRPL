classdef lineMapLocalizer13 < handle    
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
        times = [];
    end
    
    methods
    
        function obj = lineMapLocalizer13(lines_p1,lines_p2,gain,errThresh,gradThresh)            
            % create a lineMapLocalizer           
            obj.lines_p1 = lines_p1;       
            obj.lines_p2 = lines_p2;     
            obj.gain = gain;         
            obj.errThresh = errThresh;     
            obj.gradThresh = gradThresh;
            
            obj.times = [];
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
        


        function ids = throwOutliers(obj,currPose,ptsInModelFrame)            
            % Find ids of outliers in a scan.             
            worldPts = currPose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);     
            ids = find(r2 > obj.maxErr*obj.maxErr);        
        end

        function avgErr2 = fitError(obj,inPose,ptsInModelFrame)            
            % Find the variance  of perpendicular distances of            
            % all points to all lines
            % transform the points 
            
            worldPts = inPose.bToA()*ptsInModelFrame;
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
            %fprintf("jacobian error %d\n", err2_Plus0);
            eps = 1e-7;
            
            dp_x = [eps ; 0.0 ; 0.0]; 
            newPose = pose(poseIn.getPoseVec()+dp_x);
            a = newPose.x();
            b = newPose.y();
            c = newPose.th();
            d = fitError(obj, newPose, modelPts);
            e = d - err2_Plus0;
            jacX = (fitError(obj, newPose, modelPts) - err2_Plus0)/eps;
            
            dp_y = [0.0 ; eps ; 0.0];          
            newPose = pose(poseIn.getPoseVec()+dp_y);
            jacY = (fitError(obj, newPose, modelPts) - err2_Plus0)/eps;
            
            dp_th = [0.0 ; 0.0; eps];          
            newPose = pose(poseIn.getPoseVec()+dp_th);
            jacTh = (fitError(obj, newPose, modelPts) - err2_Plus0)/eps;
            
            J = [jacX, jacY, jacTh];   
            %err2_Plus0 = sqrt(err2_Plus0);    
        end
        
        function [successPts, successIters, errorStop, gradStop, outPose, tend] = refinePose(obj, inPose, pointsInModelFrame, maxIters, robotBodyPts)
            err = 100;
            grad = 100;
            errArr = [];
            currIteration = 0;
            ptsThresh = 5;
            outPose = inPose;
            tstart = tic;
            successPts = true;
            successIters = true;
            outPose = inPose;
            errorStop = false;
            gradStop = false;
            if (length(pointsInModelFrame) < ptsThresh)
                %fprintf("in here but this bad\n");
                successPts = false;
            else
                %fprintf("this is gooder\n");
                tStart = tic;
                t1 = toc(tStart);
                while((err >= obj.errThresh) && (grad >= obj.gradThresh) && (currIteration < maxIters))
                    condition2 = (grad > obj.gradThresh);
                    condition1 = (err > obj.errThresh);
                    diffgrad = grad - obj.gradThresh;
                    [err,J] = obj.getJacobian(inPose, pointsInModelFrame);
                    grad = abs(J(1) + J(2) + J(3));
                    a = inPose.x() - obj.gain*J(1);
                    b = inPose.y() - obj.gain*J(2);
                    c = inPose.th() - obj.gain*J(3);
                    
                    inPose = pose(a, b, c);
%                     fprintf("in loop pose: %d, %d, %d, J: %d, %d, %d, gain: %d\n",inPose.x(), inPose.y(), inPose.th(),...
%                         J(1), J(2), J(3), obj.gain); 
                    worldBodyPts = inPose.bToA()*pointsInModelFrame;
                    %fprintf("here\n");
                    %worldBodyPts
                    x = [0 0; 0 (1.2192*2)]; y = [0 0; (1.2192*2) 0];
%                     kh = plot(obj.lines_p1, obj.lines_p2);
%                     kh = plot(x, y);
%                     axis([-1, 3, -1, 3]); 
%                     hold on
%                     ph2 = plot(robotBodyPts(1,:), robotBodyPts(2,:));
%                     hold on
%                     kh = scatter(worldBodyPts(1,:),worldBodyPts(2,:));
%                     hold off
                    pause(0.001);
                    currIteration = currIteration + 1;
                   
                   
                end
                t2 = toc(tStart);
                loopTime = t2 - t1;
                kh = plot(x, y);
                title('Robot WTF Lidar Points With Wall');
                xlabel('X (m)');
                ylabel('Y (m)');
                axis([-1, 2, -1, 2]); 
                hold on
                ph2 = plot(robotBodyPts(1,:), robotBodyPts(2,:));
                hold on
                kh = scatter(worldBodyPts(1,:),worldBodyPts(2,:));
                hold off
                if (err < obj.errThresh)
                    errorStop = true;
                end
                if (grad < obj.gradThresh)
                    gradStop = true;
                end
                if (err >= obj.errThresh && grad >= obj.gradThresh)
                    successIters = false;
                    %fprintf("MAX ITERS CUT OFF\n");
                end
                obj.times = [obj.times, loopTime];
                avgTime = mean(obj.times);
                fprintf("errstop %d, gradStop %d, iterationstop %d: time %d, avg %d\n", errorStop, gradStop, ~successIters, loopTime, avgTime);
%                 
%                 kh = plot(obj.lines_p1, obj.lines_p2);
%                 kh = plot(x, y);
%                 axis([-1, 2, -1, 2]); 
%                 hold on
%                 ph2 = plot(robotBodyPts(1,:), robotBodyPts(2,:));
%                 hold on
%                 kh = scatter(worldBodyPts(1,:),worldBodyPts(2,:));
%                 hold off
%                 if (err < obj.errThresh)
%                     errorStop = true;
%                 end
%                 if (grad < obj.gradThresh)
%                     gradStop = true;
%                 end
%                 if (err >= obj.errThresh && grad >= obj.gradThresh)
%                     successIters = false;
%                     %fprintf("MAX ITERS CUT OFF\n");
%                 end

                if (err >= obj.errThresh && grad >= obj.gradThresh)
                    success = false;
                else
                    success = true;
                end
                outPose = inPose;
                %fprintf("refine pose time %d\n", tend);
            end
            tend = toc(tstart);

            
          
            
        end
        
        function testJacobian(obj)
            xPts = [0, 4, 0, -4, 5, -5, 0, 0, 0, 0];
            yPts = [0, 0, 4, 0, 0, 0, 5, -5, -4, 1];
            thPts = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
            pointsInModelFrame = [xPts ; yPts; thPts];
            [err, J] = obj.getJacobian(pose(1,-1,0), pointsInModelFrame);
            err
            J
            
        end
        
%         function test(obj)
%             %set up lines
%             p1 = [-2 ; -2];
%             p2 = [ 2 ; -2];
%             p3 = [ 2 ;  2];
%             p4 = [-2 ;  2];
%             obj.lines_p1 = [p1 p2 p3 p4];
%             obj.lines_p2 = [p2 p3 p4 p1];
%             
%             
%             % Set up test points
%             nPts = 10;
%             x1   = -2.0*ones(1,nPts);
%             x2 = linspace(-2.0,2.0,nPts);
%             x3 =  2.0*ones(1,nPts);
%             y1 = linspace(0.0,2.0,nPts);
%             y2 = 2.0*ones(1,nPts);
%             y3 = linspace(2.0,0,nPts);
%             w  = ones(1,3*nPts);
%             
%             x1pts = [x1 x2 x3];
%             y1pts = [y1 y2 y3];
%             w1pts = w;
%             
%             pointsInModelFrame = [x1pts ; y1pts ; w1pts];
%             
%             % pick a pose
%             dx = -0.05*rand();
%             dy = -0.05*rand();
%             dt = -0.05+0.2*rand();
%             thePose = pose(0.0+dx,0.0+dy,0.0+dt);
%             ids = throwOutliers(obj, pose, pointsInModelFrame);
%             
%             allIds = linspace(1, length(pointsInModelFrame), length(pointsInModelFrame));
%             goodIds = setdiff(allIds, ids);
%             pointsInModelFrame = pointsInModelFrame(:, goodIds);
%             
%             refinePose(obj, thePose, pointsInModelFrame, 10);
%             
%         end
%     
    
    end
end

    
        
    
    
    