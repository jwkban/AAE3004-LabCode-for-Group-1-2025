%Ex 1 for Task 1
rplid=rossubscriber('/scan','sensor_msgs/LaserScan','DataFormat','struct');
scan=receive(rplid,10);
Lidscan=rosReadLidarScan(scan);
formatSpec='Angle (Front): %10.5f\nDistance (Front): %10.5f\n';
fprintf(formatSpec, rad2deg(Lidscan.Angles(1)), Lidscan.Ranges(1))
formatSpec='Angle Increment (deg): %f\n';
fprintf(formatSpec, rad2deg(scan.AngleIncrement))
x1=floor(5/rad2deg(scan.AngleIncrement)); %Read the angle index at -5°
formatSpec='Corresponding output distance at angle -5° measured CCW from front of the lidar: %.5f\n';
fprintf(formatSpec, scan.Ranges(x1)) %Output the distance from first wall/obstacles measured by the lidar/radar from front of the lidar -5°
x=floor(355/rad2deg(scan.AngleIncrement)); %Read the angle index at +5°
formatSpec='Corresponding output distance at angle +5° measured CCW from front of the lidar: %.5f\n';
fprintf(formatSpec, scan.Ranges(x)) %Output the distance from first wall/obstacles measured by the lidar/radar from front of the lidar +5°


