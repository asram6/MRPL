robot = raspbot('Raspbot-0');
robot.startLaser();
pause(3);
while true
    ranges = robot.laser.LatestMessage.Ranges;
    plot(ranges)
    pause(0.2)
end
robot.stopLaser();

