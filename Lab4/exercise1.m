goaldist = 0.1;
robot =  raspbot('sim');
encoderStart = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
encoderCurr = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
e = (goaldist)-(encoderCurr-encoderStart);
%nooooo get from encoder
kp = 1; kd = 0; ki = 0;
%upid = kp * e + kd * derivative(e) + kp * integral of e dt
tstart = tic;
tcurr = toc(tstart);
eint = 0;
eintmax = .1;
upid = 0;
distErr = [];
time = [];
while (1) 
    while ((tcurr < 6) && (abs(e) > .0001))
        fprintf('%d %d  upid = %d\n', tcurr, e, upid);
        olde = e;
        oldt = tcurr;
        encoderCurr = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
        e = (goaldist)-(encoderCurr-encoderStart);
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
        if (abs(upid) > 0.3)
            if (upid > 0)
                upid = 0.3;
            else
                upid = -1 * 0.3;
            end
        end
        robot.sendVelocity(upid,upid);
        distErr = [distErr, e];
        time = [time, tcurr];
    end
    if (goaldist = .1) 
        goaldist = 0;
    else
        goaldist = .1
    end
end
plot(distErr, time);

