classdef simRobot1 < handle
    
    properties
        
        x, y, theta, prevEncoderX, prevEncoderY, tPrev, vl, vr;
        
    end
    
    
    
    methods
        
        function obj = simRobot1(encoderX, encoderY)
            obj.prevEncoderX = encoderX;
            obj.prevEncoderY = encoderY;
            obj.x = 0; obj.y = 0; obj.theta = 0;
            obj.tPrev = 0; obj.vl = 0; obj.vr;
        end
        
        function x = getX(obj)
            x = obj.x;
        end
        
        function integrate(obj, encoderX, encoderY, tCurr)
            dt = tCurr - obj.tPrev;
            obj.vl = (encoderX - obj.prevEncoderX)/dt; 
            obj.vr = (encoderY - obj.prevEncoderY)/dt;
            vactual = (obj.vl + obj.vr) /2;
            omegaActual = (obj.vr - obj.vl) / 0.087; %%!!!!!!!!!!<-------
            obj.theta = obj.theta + omegaActual * dt;
            %fprintf("before obj.theta = %d\n", obj.theta);
            while (obj.theta >= pi)
                obj.theta = obj.theta-2*pi;
            end
            while (obj.theta < -pi)
                obj.theta = obj.theta+2*pi;
            end
            %fprintf("after obj.theta = %d\n", obj.theta);
            obj.x = obj.x + vactual*cos(obj.theta)*dt;
            obj.y = obj.y + vactual*sin(obj.theta)*dt;
            obj.tPrev = tCurr;
            obj.prevEncoderY = encoderY;
            obj.prevEncoderX = encoderX; 
            
            %fprintf("encoderX %d, encoderY %d, dt %d, vactual %d, omegaActual %d, theta %d, x %d, y %d\n", encoderX, encoderY, dt, vactual, omegaActual, obj.theta, obj.x, obj.y);
            
        end
        
    end
    
end