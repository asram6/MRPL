robot = raspbot('Raspbot-09');
robot.sendVelocity(.050, .050);
robot.startLaser();
robot.encoders.NewMessageFcn = @encoderEventListener;
pause(10)
robot.sendVelocity(0, 0);
robot.shutdown();
