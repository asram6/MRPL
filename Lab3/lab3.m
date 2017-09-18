
   global preval;
   global currval;
   preval = false;
   currval = false;
   vt = 0.2; sf = 1; tf = sf/vt;
   ktheta = 2*pi/sf; kk = 15.1084;
   ks = 3; Tf = ks * tf;
   tstart = tic;
   t = toc(tstart);
   vlArr = []; vrArr = []; dtArr = [];
   xArr = []; yArr = [];
   robot = raspbot("Raspbot-9");
   fprintf("105\n");
   robot.encoders.NewMessageFcn = @encoderEventListener;
   x = 0;
   y = 0;
   th = 0;
   while(t < Tf) 
       fprintf("in loop\n");
       while (preval == currval)
           pause(0.001);
       end
       fprintf("out of loop\n");
       preval = currval;
       encoderx = robot.encoders.LatestMessage.Vector.X;
       encodery = robot.encoders.LatestMessage.Vector.Y;
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
       plot(yArr, xArr);
       T = toc(tstart);
       dt = (T - t);
       th = th + omegat*dt;
       x = x + vt*cos(th)*dt;
       y = y + vt*sin(th)*dt;
       xArr = [xArr, x];
       yArr = [yArr, y];
       plot(xArr, yArr);
       t = T;
       pause(0.1);
       fprintf("%d %d\n", vl, vr);
   end
   robot.encoders.NewMessageFcn = [];
   fprintf("%d \n", length(vlArr));
   %exercise3.modelDiffSteerRobot(vlArr, vrArr, 0, Tf, dtArr); 
  
 
 function encoderEventListener(handle,event)
           fprintf("HERE\n");
           global currval;
           currval = ~currval;
           encoderDataTimestamp = double(event.Header.Stamp.Sec);
end
