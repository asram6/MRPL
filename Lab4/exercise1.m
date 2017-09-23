goaldist = 0.1;
robot =  raspbot('sim');
encoderStart = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
encoderCurr = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
e = abs((goaldist)-(encoderCurr-encoderStart));
%nooooo get from encoder
kp = 10; kd = 0; ki = 0;
%upid = kp * e + kd * derivative(e) + kp * integral of e dt
tstart = tic;
tcurr = toc(tstart);
eint = 0;
eintmax = .1;
while ((tcurr < 6) && (e > .0001))
    olde = e;
    oldt = tcurr;
    e = abs((goaldist)-(encoderCurr-encoderStart));
    tcurr = toc(tstart);
    
    dedt = (e-olde)/(tcurr-oldt);
    eint = eint + (e * (tcurr-oldt));
    if (abs(eint) > eintmax)
        if (eint > 0)
            eint = eintmax;
        else
            eint = -1 * eintmax;
        end
    end
    upid = kp*e + kd*dedt + ki * eint;
    robot.sendVelocity(upid,upid);
end

