%% Lab 1 Task 1 Move the Robot

robot = raspbot('Raspbot-09');
tstart = tic;
leftEncoder = robot.encoders.LatestMessage.Vector.Y;
rightEncoder = robot.encoders.LatestMessage.Vector.X;
timeArray = zeros(1,1);
leftArray = zeros(1,1);
rightArray = zeros(1,1);
averageMeters = 0;
while averageMeters < 0.3048
    robot.sendVelocity(0.05, 0.05);
    differenceLeft = abs(robot.encoders.LatestMessage.Vector.Y - leftEncoder);
    differenceRight = abs(robot.encoders.LatestMessage.Vector.X - rightEncoder);
    averageMeters = (differenceLeft + differenceRight) / 2;
    timeArray = [timeArray toc(tstart)];
    leftArray = [leftArray differenceLeft*100];
    rightArray = [rightArray differenceRight*100];
    plot(timeArray, leftArray, timeArray, rightArray);
    pause(0.1);
end

robot.sendVelocity(0,0);
pause(1)

leftEncoder = robot.encoders.LatestMessage.Vector.Y;
rightEncoder = robot.encoders.LatestMessage.Vector.X;
averageMeters = 0;
while averageMeters < 0.3048
    robot.sendVelocity(-0.05, -0.05);
    differenceLeft = abs(robot.encoders.LatestMessage.Vector.Y - leftEncoder);
    differenceRight = abs(robot.encoders.LatestMessage.Vector.X - rightEncoder);
    averageMeters = (differenceLeft + differenceRight) / 2;
    timeArray = [timeArray toc(tstart)];
    leftArray = [leftArray differenceLeft*100];
    rightArray = [rightArray differenceRight*100];
    plot(timeArray, leftArray, timeArray, rightArray);
    pause(0.1);
end

robot.sendVelocity(0,0);