robot = raspbot('Raspbot-09');
robot.encoders.NewMessageFcn = @encoderEventListener;
while true
    robot.sendVelocity(.05, .05);
    pause(0.05);
end


