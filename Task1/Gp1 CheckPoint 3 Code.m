%Obtain Lidar Data
rplid=rossubscriber('/scan','DataFormat','struct');
scan=receive(rplid,10);

% Extract "angle" & "distance"  from /scan message
Lidscan=rosReadLidarScan(scan);

%Print out required shown value from the Command Prompt output
formatSpec='Angle (Front): %10.5f\nDistance (Front): %10.5f\n';
fprintf(formatSpec, rad2deg(Lidscan.Angles(1)), Lidscan.Ranges(1))
formatSpec='Angle Increment (deg): %f\n';
fprintf(formatSpec, rad2deg(scan.AngleIncrement))
minAngle = min(rad2deg(Lidscan.Angles(:)));
maxAngle = max(rad2deg(Lidscan.Angles(:)));
formatSpec='Angle Min (deg): %10.6f\nAngle Max (deg): %10.6f\n';
fprintf(formatSpec, minAngle, maxAngle)
minValue = scan.RangeMin();
formatSpec = 'Range Min [m]: %10.6f\n';
fprintf(formatSpec, minValue);
maxValue =  scan.RangeMax();
formatSpec = 'Range Max [m]: %10.6f\n';
fprintf(formatSpec, maxValue);

%Checkpoint 3
