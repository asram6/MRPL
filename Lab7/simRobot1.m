classdef simRobot1 < handle
    
    properties
        
        x, y, theta, prevEncoderX, prevEncoderY, tPrev, vl, vr,
        prevEncoderTime;
        
    end
    
    
    
    methods
        
        function obj = simRobot1(encoderX, encoderY)
            obj.prevEncoderX = encoderX;
            obj.prevEncoderY = encoderY;
            obj.prevEncoderTime = 0;
            obj.x = 0; obj.y = 0; obj.theta = 0;
            obj.tPrev = 0; obj.vl = 0; obj.vr;
        end
        
        function x = getX(obj)
            x = obj.x;
        end
        
        function integrate(obj, encoderX, encoderY, tCurr, encoderTime)
            dt2 = encoderTime - obj.prevEncoderTime;
            dt = tCurr - obj.tPrev;
            if dt2 ~= 0
                obj.vl = (encoderX - obj.prevEncoderX)/dt2; 
                obj.vr = (encoderY - obj.prevEncoderY)/dt2;
                vactual = (obj.vl + obj.vr) /2;
                omegaActual = (obj.vr - obj.vl) / robotModel.W; 
                obj.theta = obj.theta + omegaActual * dt /2;
                obj.theta = atan2(sin(obj.theta), cos(obj.theta));
                obj.x = obj.x + vactual*cos(obj.theta)*dt;
                obj.y = obj.y + vactual*sin(obj.theta)*dt;
                obj.theta = obj.theta + omegaActual * dt /2;
                obj.theta = atan2(sin(obj.theta), cos(obj.theta));
                obj.tPrev = tCurr;
                obj.prevEncoderY = encoderY;
                obj.prevEncoderX = encoderX; 
                obj.prevEncoderTime = encoderTime;
            end
            
            
            %fprintf("encoderX %d, encoderY %d, dt %d, vactual %d, omegaActual %d, theta %d, x %d, y %d\n", encoderX, encoderY, dt, vactual, omegaActual, obj.theta, obj.x, obj.y);
            
        end
        
    end
    
end