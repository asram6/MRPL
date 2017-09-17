function encoderEventListener(handle,event)
    %EncoderEventListener Invoked when new encoder data arrives. 
    % A MATLAB event listener for the Robot. Invoked when
    % encoder data arrives.
    fprintf("HERE");
    fprintf('%d', 1);
    persistent tPrev;
    persistent encoderPrev;
    persistent robot;
    if isempty(tPrev) 
        fprintf("in tprev")
        tPrev = tic;
    end
    if isempty(robot)
        fprintf("in robot")
        robot = raspbot('Raspbot-09');
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
    plot(tDiff, velocity);
end