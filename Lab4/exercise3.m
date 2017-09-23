goaldist = 0.1;
robot =  raspbot('Raspbot-9');
encoderStart = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
encoderCurr = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
e = (goaldist)-(encoderCurr-encoderStart);
%nooooo get from encoder
kp = 5; kd = 0.01; ki = 0;
%upid = kp * e + kd * derivative(e) + kp * integral of e dt
tstart = tic;
tcurr = toc(tstart);
eint = 0;
eintmax = .1;
uref = 0;
distErr = [];
time = [];
oldoldt = 0; 
sgn = 1;
while (1) 
    while ((tcurr < 6) && (abs(e) > .0001))
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
        uref = trapezoidalVelocityProfile(tcurr , 3 * 0.25, 0.25, goaldist, sgn);
        if (abs(uref) > 0.3)
            if (uref > 0)
                uref = 0.3;
            else
                uref = -1 * 0.3;
            end
        end
        robot.sendVelocity(uref,uref);
        pause(0.1);
        distErr = [distErr, e];
        time = [time, oldoldt + tcurr];
        plot(time, distErr);
    end    
    if (goaldist == .1) 
        sgn = -1;
        goaldist = 0;
    else
        sgn = 1;
        goaldist = .1;
    end
    
    oldoldt = oldoldt + tcurr;
    tstart = tic;
    tcurr = toc(tstart);
    encoderCurr = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
    e = (goaldist)-(encoderCurr-encoderStart);
end