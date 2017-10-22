goaldist = 1;
robot =  raspbot('Raspbot-9');
pause(1);
encoderStart = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
%nooooo get from encoder
kp = 3; kd = .01; ki = 0;
%upid = kp * e + kd * derivative(e) + kp * integral of e dt

tstart = tic;
tcurr = toc(tstart);
eint = 0;
eintmax = .1;
uref = 0;
distErr = [];
graphyThing = [];
time = [];
oldoldt = 0; 
sgn = 1;
uref = .1;
distActual = 0;
sref = 0;
tdelay = 0;
sdelay = 0;
graph2 = [];
flag = 1; e = 0;
preval = false;
currval = false;
while (tcurr < (tdelay + 1 + 4.333333))

       
       
    olde = e;
    oldt = tcurr;
    tcurr = toc(tstart);
    
    uref = trapezoidalVelocityProfile(tcurr, 3 * 0.25, 0.25, goaldist, sgn);
    udelay = trapezoidalVelocityProfile(tcurr - tdelay, 3 * 0.25, 0.25, goaldist, sgn);
    sref = sref + uref * (tcurr-oldt);
    sdelay = sdelay + udelay*(tcurr-oldt);
    
    encoderCurr = (robot.encoders.LatestMessage.Vector.X + robot.encoders.LatestMessage.Vector.Y) / 2;
    e = (sdelay)-(encoderCurr-encoderStart);
    %fprintf("e = %d\n", e);
    distActual = encoderCurr-encoderStart;
    dedt = (e-olde)/(tcurr-oldt);
    eint = eint + (e * (tcurr-oldt));
    if (abs(eint) > eintmax)
        if (eint > 0)
            eint = eintmax;
        else
            eint = -1 * eintmax;
        end
    end
    if (flag)
        upid = kp*e + kd*dedt + ki * eint;
        if (abs(upid) > 0.3)
            if (upid > 0)
                upid = 0.3;
            else
                upid = -1 * 0.3;
            end
        end
    else
        upid = 0;
    end
    
    u = upid + uref;
    robot.sendVelocity(u,u);
    pause(0.001);
    distErr = [distErr, distActual];
    graphyThing = [graphyThing, sdelay];
    graph2 = [graph2, sdelay-distActual];
    time = [time, oldoldt + tcurr];
end    
pause(1);
plot(time, distErr, time, graphyThing);
legend('actual distance', 'sdelay');
title("Delayed Reference Distance and Actual Distance on Robot 9");
       ylabel('Distance Traveled (m)');
       xlabel('Time (seconds)');
figure;
plot(time, graph2);
title("Error vs Time");
       xlabel('Time (seconds)');
       ylabel('Error (m)');
uref = 0;
robot.sendVelocity(uref,uref);


function encoderEventListener(handle,event)
    global currval;
    currval = ~currval;
    encoderDataTimestamp = double(event.Header.Stamp.Sec);
end
