classdef simRobot1
    
    properties
        
        x, y, theta, prevEncoderX, prevEncoderY, tPrev, vl, vr;
        
    end
    
    
    
    methods
        
        function obj = simRobot1(encoderX, encoderY)
            %obj.prevEncoderX = encoderX;
            %obj.prevEncoderY = encoderY;
            obj.x = 0; obj.y = 0; obj.theta = 0;
            obj.tPrev = 0; obj.vl = 0; obj.vr;
        end
        
        function integrate(obj, encoderX, encoderY, tCurr)
            dt = tCurr - obj.tPrev;
            obj.vl = (encoderX - obj.prevEncoderX)/dt; 
            obj.vr = (encoderY - obj.prevEncoderY)/dt;
            vactual = (obj.vl + obj.vr) /2;
            omegaActual = (obj.vr - obj.vl) / 0.088;
            obj.theta = obj.theta + omegaActual * dt;
            obj.x = obj.x + vactual*cos(obj.theta)*dt;
            obj.y = obj.y + vactual*sin(obj.theta)*dt;
            obj.tPrev = tCurr;
            obj.prevEncoderX = encoderX; obj.prevEncoderY = encoderY;
            
        end
        
    end
    
end