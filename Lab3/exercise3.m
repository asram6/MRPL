classdef exercise3
   properties
      PropertyName
   end
   methods(Static)
       function [x, y, th] = modelDiffSteerRobot(vl, vr, t0, tf, dt)
           V = (vl+vr)/2;
           omega = (vr - vl)/0.085;
           t = t0;
           th = 0;
           x = 0;
           y = 0;
           while t < tf
               %fprintf("in steer loop %d %d\n", t, tf);
               th = th + omega*dt/2;
               x = x + V*cos(th)*dt;
               y = y + V*sin(th)*dt;
               th = th + omega*dt/2;
               t = t + dt;
               pause(0.1);
           end
       end
       
       function cornuSpiral()
           v = 0.1;
           k = 1/8;
           t = 0;
           dt = 0.001;
           tf = sqrt(32*pi);
           xArr = [];
           yArr = [];
           while t < tf
               %fprintf("in cornu spiral loop %d %d\n", t, tf);
               omega = k*t;
               vr = v + (0.085/2)*omega;
               vl = v - (0.085/2)*omega;
               [x y th] = exercise3.modelDiffSteerRobot(vl, vr, 0, t, dt);
               xArr = [xArr, x];
               yArr = [yArr, y];
               fprintf("x y th %d %d %d\n", x, y, th); 
               plot(xArr, yArr);
               axis([0 0.4 0 0.4]);
               t = t + dt;
               pause(0.1);
           end
       end
    
   end
end