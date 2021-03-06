classdef exercise3
   properties
      PropertyName
   end
   methods(Static)
       function [x, y, th] = modelDiffSteerRobot(vl, vr, t0, tf, dt)
           th = 0;
           x = 0;
           y = 0;
           xArr = [];
           yArr = [];
           myPlot = plot(xArr, yArr, 'b-');
           xlim([0.0, 0.5]);
           ylim([0.0, 0.5]);
           for i = (1:length(vr))
               V = (vl(i)+vr(i))/2;
               omega = (vr(i) - vl(i))/0.085;
               th = th + omega*dt(i)/2;
               while (th > pi)
                   th = th - 2*pi;
               end
               while (th < -pi)
                   th = th + 2*pi;
               end
               x = x + V*cos(th)*dt(i);
               y = y + V*sin(th)*dt(i);
               fprintf("in steer loop %d %d\n", x, y);
               xArr = [xArr, x];
               yArr = [yArr, y];
               pause(0.005);
%                g = sprintf('%d ', [get(myPlot, 'xdata') x]);
%                fprintf("vector %s -- \n", g);
%                set(myPlot, 'xdata', xArr, 'ydata', yArr);
%                myPlot;
               plot(xArr, yArr);
               %axis([0 .01 0 .01]);
               th = th + omega*dt(i)/2;
           end
           
       end
       
       function cornuSpiral()
           v = 0.1;
           k = 1/8;
           t = 0;
           dt = 0.001;
           tf = sqrt(32*pi);
           vr = [];
           vl = [];
           while t < tf
               %fprintf("in cornu spiral loop %d %d\n", t, tf);
               omega = k*t;
               vr = [vr, v + (0.085/2)*omega];
               vl = [vl, v - (0.085/2)*omega];
               t = t + dt;
           end
           fprintf("%d \n", length(vr));
           [x y th] = exercise3.modelDiffSteerRobot(vl, vr, 0, t, dt);
       end
           
       function exercise2(robot)
            persistent tDiffArr;
            persistent velArr;
            persistent tPrev;
            persistent encoderPrev;
            if isempty(tDiffArr) 
                tDiffArr = [];
            end
            if isempty(velArr) 
                velArr = [];
            end
            if isempty(tPrev) 
                fprintf("in tprev")
                tPrev = tic;
            end
            if isempty(encoderPrev)
                fprintf("in encoderpev")
                encoderPrev = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y)/2;
            end
            tDiff = toc(tPrev);
            x = robot.encoders.LatestMessage.Vector.X;
            y = robot.encoders.LatestMessage.Vector.Y;
            encoderNow = (x+y)/2;
            encoderDiff = encoderNow - encoderPrev;
            velocity = encoderDiff/tDiff;
            tDiffArr = [tDiffArr, tDiff];
            velArr = [velArr, velocity];
            fprintf("tDiff = %d     velocity = %d \n", tDiff, velocity);
            plot(tDiffArr, velArr);
       end
       
       function figure8()
           vt = 0.2; sf = 1; tf = sf/vt;
           ktheta = 2*pi/sf; kk = 15.1084;
           ks = 3; Tf = ks * tf;
           tstart = tic;
           t = toc(tstart);
           vlArr = []; vrArr = []; dtArr = [];
           xArr = []; yArr = [];
           robot = raspbot("Rasbot-9");
           xprev = robot.encoders.LatestMessage.Vector.X;
           yprev = robot.encoders.LatestMessage.Vector.Y;
           while(t < Tf) 
               st = vt*t/ks;
               angle = ktheta*st;
               while (angle > pi)
                   angle = angle - 2*pi;  
               end
               while (angle < -1 * pi)
                   angle = angle + 2*pi;  
               end
               k = (kk/ks)*sin(angle);
               omegat = k*vt;
               vr = vt + (0.0425) * omegat;
               vl = vt - (0.0425) * omegat;
               vlArr = [vlArr, vl];
               vrArr = [vrArr, vr];
               robot.sendVelocity(vl, vr);
               x = robot.encoders.LatestMessage.Vector.X - xprev;
               y = robot.encoders.LatestMessage.Vector.Y - yprev;
               xArr = [xArr x];
               yArr = [yArr -y];
               plot(yArr, xArr);
               T = toc(tstart);
               dtArr = [dtArr, (T - t)];
               t = T;
               pause(0.1);
               fprintf("%d %d\n", vl, vr);
           end
           fprintf("%d \n", length(vlArr));
           %exercise3.modelDiffSteerRobot(vlArr, vrArr, 0, Tf, dtArr); 
       end
       
   end
end