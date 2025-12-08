%Ex 2 for Task 1

[pub,msg]=rospublisher("/cmd_vel",'geometry_msgs/Twist'); 
msg.Linear.X=1.0; % Set the initial speed of the car to allow it to move straight forward
msg.Angular.Z=0; % No initial turning of the Car occurs

desiredRate=100;
rate=robotics.Rate(desiredRate);
rate.OverrunAction='drop';
reset(rate)

y=0.15 % Set the stopping distance when the distance of the center of the lidar and the wall/ object infront is less than this value . While we are group 1, will be 1*0.15m
while rate.TotalElapsedTime<Inf  
    rplid=rossubscriber('/scan','sensor_msgs/LaserScan','DataFormat','struct');
    scan=receive(rplid,100);
    Lidscan=rosReadLidarScan(scan);
    x=Lidscan.Ranges(1) % Read the distance measured directly infront of the center of the lidar and the first wall/object/obstacles infront of the car. ( Read at anytime when the car is moving. This means the distance measured is always higher than the stopping distance)
    if x>y
     send(pub,msg)
     waitfor(rate);
    else 
        break % Stop the While loop if the distance between the Car and the Wall/object infront is lower than the stopping distance
    end
end

msg.Linear.X=0.0; % Stop the car when the PID program is not in the While loop, that is within the stopping distance 
msg.Angular.Z=0.0;
send(pub,msg)

statistics(rate);

