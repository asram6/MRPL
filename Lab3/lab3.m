
   global preval;
   global currval;
   preval = false;
   currval = false;
   robot = raspbot("Raspbot-9");
   pause(3);
   vt = 0.2; sf = 1; tf = sf/vt;
   ktheta = 2*pi/sf; kk = 15.1084;
   ks = 3; Tf = ks * tf;
   tstart = tic;
   t = toc(tstart);
   vlArr = []; vrArr = []; dtArr = [];
   xArr = []; yArr = [];
   fprintf("105\n");
   robot.encoders.NewMessageFcn = @encoderEventListener;

   prevx = robot.encoders.LatestMessage.Vector.X;
   prevy = robot.encoders.LatestMessage.Vector.Y;
   x = 0;
   y = 0;
   th = 0;
   while(t < Tf) 
       while (preval == currval)
           pause(0.001);
       end
       preval = currval;
       
       st = vt*t/ks;
       angle = ktheta*st;
       while (angle > pi)
           angle = angle - 2*pi;  
       end
       while (angle < -1 * pi)
           angle = angle + 2*pi;  
       end
       angle = angle; %error scaling
       k = (kk/ks)*sin(angle);
       omegat = k*vt;
       vr = vt + (0.044) * omegat;
       vl = vt - (0.044) * omegat;
       vlArr = [vlArr, vl];
       vrArr = [vrArr, vr];
       robot.sendVelocity(vl, vr);
       plot(yArr, xArr);
       T = toc(tstart);
       encoderx = robot.encoders.LatestMessage.Vector.X;
       encodery = robot.encoders.LatestMessage.Vector.Y;
       diffx = encoderx - prevx;
       diffy = encodery - prevy;
       prevx = encoderx;
       prevy = encodery;
       dt = (T - t);
       vlactual = diffx/dt;
       vractual = diffy/dt;
       Vactual = (vlactual+vractual)/2;
       omegaActual = (vractual-vlactual)/.088;
       th = th + omegaActual*dt;
       x = x + Vactual*cos(th)*dt;
       y = y + Vactual*sin(th)*dt;
       xArr = [xArr, x];
       yArr = [yArr, y];
       plot(xArr, yArr);
       title("Robot Encoder Data Moving in Figure 8");
       xlabel('X(m)');
       ylabel('Y(m)');
       t = T;
       pause(0.1);
   end
   robot.encoders.NewMessageFcn = [];
   robot.sendVelocity(0,0);
   fprintf("%d \n", length(vlArr));
   %exercise3.modelDiffSteerRobot(vlArr, vrArr, 0, Tf, dtArr); 
  
 
 function encoderEventListener(handle,event)
           global currval;
           currval = ~currval;
           encoderDataTimestamp = double(event.Header.Stamp.Sec);
end
