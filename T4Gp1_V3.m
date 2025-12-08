% Performance Test (Task 4) from Group 1 with Car 20 
% Group member name:  AL AKIB Ahmad Munjir   BAN Wenrui   CHAI Yuan   CHAN Man Ho

% 2025-11-11(By Ban Wenrui)
% Used the basic code background fundation from task 3 , 
% Included parameter based on the Task requirement 
%Add method to allow the program to randomly generate a k_d and k_p value
%from given range provided by the test 

%2025-11-18 (By Ban Wenrui)
% Update the selection method for k_d and k_p values to allow the respective  values to be
% selected from the pool given based on the email 
%Improved on the performance by dropping certain conditions, ensure higher
%smooth and stability
%Allow the program to self generate/print the kp and kd used for each run.
%Code now should be ready for testing 

%Define user constants
a_ = deg2rad(360-40);
b_ =  deg2rad(360-90); %(use radian) 
% selection of two target reference points from the radar scan one at 40
% degree and one at 90 degree cw measured from 0 degree point
% variable here set for radar scan coordinate from -180 to 180
s = 0.4; 
d_star = 0.5;
L = 0.145; % (measure the length on the robot car in METERS)
%k_p=1.2
%k_d=0.005 % Value used for task 3, 
kp_pool=[0.6, 1.2, 1.8];
kd_pool=[0,0.005];
k_p=kp_pool(randi(numel(kp_pool))); % will randomly generate value from pool of 0.6 1.2 1.8 as given by email 
k_d=kd_pool(randi(numel(kd_pool))); % will randomly generate value  from pool of 0 0.005  as given by email
et_1 = d_star-0.25; %(previous error) This is the start error as we set d_initial as 0.25m
start_time = seconds(rostime('now'));
t0 =start_time ; % record the start time

%Set up subscriber (/scan) & publisher (/cmd_vel)
robotCmd = rospublisher("/cmd_vel","DataFormat","struct"); 
velMsg = rosmessage(robotCmd);
laser = rossubscriber("/scan","DataFormat","struct");

%Compute linear speed and angular speed
while true
    [smsg, flag] = receive(laser,5);
    Lidscan=rosReadLidarScan(smsg);
    x_front=Lidscan.Ranges(1); % Obtain front distance
    % Write code to break the while loop if no lidar data received ,
    % running time higher than 30, and object is less than 0.5meter in the front of the car
    if (flag == 0) || (x_front < 0.5) || (seconds(rostime('now'))-start_time >30)
        velMsg.Linear.X = 0.0;
        velMsg.Angular.Z = 0.0;
        send(robotCmd,velMsg);
        break
    end
    % Compute current and lookahead configuration
    i_a = floor(a_/smsg.AngleIncrement);
    i_b = floor(b_/smsg.AngleIncrement); %index
    try  % Prevent random error occuring during operation that cause the car to lose control
     a =smsg.Ranges(i_a);
     b =smsg.Ranges(i_b);  
     if isinf(a) || isinf(b)  %Prevent the car from moving if the scan distance of a and b is infinity while ensure the while loop is still operating
         velMsg.Linear.X = 0.0;
         velMsg.Angular.Z = 0.0;
         %send(robotCmd, velMsg) % this condition is dropped as it will unstable to
         %the cars performance on smoothness
         continue  
     else
       alpha = atan((a*cos(abs(b_-a_))-b)/(a*sin(abs(b_-a_)))); % alpha value should always kept postive as the car is not going to turn 180 degrees
       d = b*cos(alpha); %get the normal distance from the wall at anypoint
     % PD controller algorithm to obtain steering angle
       bar_d=d+s*sin(alpha);
       et = d_star-bar_d; % error now
       t1 = seconds(rostime('now'))-t0; %delta time
       PD=k_p*et+k_d*((et-et_1)/(t1)); % PD functions algorithm for steering_angle
       steering_angle = PD; % we assume negetive turn to right, postivie turn to left
       et_1 = et; %Update the previous error
       t0 = seconds(rostime('now')); %Update the previous time
     end
    %Publish forward speed & angular speed with stability control
    % Adjust the forward speed if the steering angle too large for
    % stability reason base on the given suggestion value and real field
    % testing. Higher smoothness performance is expected with such value
    % used
    % Use the bicycle model formula to estimate the angular speed
    % Set upper limit for velMsg.Linear.Z
     if (abs(steering_angle) > deg2rad(10))
        velMsg.Linear.X = 0.25;
        x=0.25; % Set the linear speed for angular movement usage
        %limit the angular movement of the wheel to restricted between -22
        %to 22 degrees
        if steering_angle >= deg2rad(22) 
           velMsg.Angular.Z = double(x * tan(deg2rad(22)) * (1 / L));
        elseif  steering_angle <= -deg2rad(22)
            velMsg.Angular.Z = double(x * tan(-deg2rad(22)) * (1 / L));
        else
            velMsg.Angular.Z = double(x * tan(steering_angle) * (1 / L));
        end
     elseif abs(steering_angle) >= deg2rad(5) && abs(steering_angle) <= deg2rad(10)
        velMsg.Linear.X = 0.325;
        x = 0.325;
        if steering_angle >= deg2rad(22)
            velMsg.Angular.Z = double(x * tan(deg2rad(22)) * (1 / L));
        elseif  steering_angle <= -deg2rad(22)
            velMsg.Angular.Z = double(x * tan(-deg2rad(22)) * (1 / L));
        else
            velMsg.Angular.Z = double(x * tan(steering_angle) * (1 / L));
        end
     elseif abs(steering_angle) < deg2rad(5)
        velMsg.Linear.X = 0.4;
        x=0.4;
        if steering_angle >= deg2rad(22)
           velMsg.Angular.Z = double(x * tan(deg2rad(22)) * (1 / L));
        elseif  steering_angle <= -deg2rad(22)
            velMsg.Angular.Z = double(x * tan(-deg2rad(22)) * (1 / L));
        else
            velMsg.Angular.Z = double(x * tan(steering_angle) * (1 / L));
        end
      end
     catch ME % Stop the car from moving when error occurs and continue for next iteration
         velMsg.Linear.X = 0.0;
         velMsg.Angular.Z = 0.0;
         send(robotCmd, velMsg)
        continue 
    end
send(robotCmd, velMsg) % Send the request to the robot.
end
fprintf('Kp used = %.5f, Kd used = %.5f\n', k_p, k_d);  % Print the used Kp and Kd for each trials
end_time = seconds(rostime('now'));
tot_time = end_time - start_time % generate the total time taken for  the trial