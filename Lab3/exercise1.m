<<<<<<< HEAD
robot = raspbot('Raspbot-09');
robot.sendVelocity(.050, .050);
robot.encoders.NewMessageFcn = @encoderEventListener;
=======
robot = raspbot('Raspbot-09');
robot.sendVelocity(.050, .050);
robot.startLaser();
robot.encoders.NewMessageFcn = @encoderEventListener;
pause(10)
robot.sendVelocity(0, 0);
robot.shutdown();
>>>>>>> a2790b87750cd09b8b8c8463a4a183dad957e81d
