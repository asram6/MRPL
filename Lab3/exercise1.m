global robot
robot = raspbot('sim');
pause(2);
robot.encoders.NewMessageFcn = @encoderEventListener;
pause(2);
tstart = tic;
while true
    robot.sendVelocity(.05, .05);
    pause(0.1);
end
robot.sendVelocity(0, 0);
robot.encoders.NewMessageFcn = [];
pause(0.05);

function encoderEventListener(handle,event)
    global robot
    exercise3.exercise2(robot);
end