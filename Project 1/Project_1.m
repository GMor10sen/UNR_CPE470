%Gabriel Mortensen
%Feb. 2022 
%CPE 470 
%Project 1 

function Project_1()

%Initialization for clearing all input 
clc, close all
clear 

%Obtain data via reading .txt 
[time, data] = rtpload('EKF_DATA_circle.txt');  

%Calculate total length of data for future reference (A.K.A total time)
total= 1:length(data.O_x);

%Declare variables 
reply = 0; %obtains user input 
new_reply = 0;  %also obtains user input
min_input = -1; %user input should they desire a period of noise or measuing covariation
max_input = -1; %user input should they desire a period of noise or measuing covariation
extra_covariance_query = -1;%In case user wants to do both GPS and GPS Covariance portion manipulation (not full data)
min_input_extra_GPS_co = -1; %extra range if user  wants to do both GPS and GPS Covariance portion manipulation (not full data)
max_input_extra_GPS_co = -1; %extra range if user  wants to do both GPS and GPS Covariance portion manipulation (not full data)


%Obtain User Input 
while (reply < 1 || reply > 4  )
    disp('!!!Gabriel Mortensen Project 1 CPE 470!!!')
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    disp('What would you like to do?')
    disp('  1: Perform Kalman filter and output a smooth and more accurate pose (position and orientation) of the robot from EKF_DATA_circle.txt.')
    disp('  2: Change the covariance of the sensor data (GPS) and check the output of the KF, then plot.')
    disp('  3: Change the covariance of the sensor data (IMU) and check the output of the KF, then plot.')
    disp('  4: Add noise to GPS position with changed covariance, then plot.')
    reply = input('');
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
end
%If user wants to do anything other than normal KF, get more input 
if (reply > 1 && reply < 5)
     while(new_reply < 1 || new_reply > 2)
         disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
         disp('Where would you like to add the noise?')
         disp(' 1. Add noise to the entire data set')
         disp(' 2. Add noise to certain periods of data')
         new_reply = input('');
         disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
     end

    %Obtain the standard deviation from the user 
     disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
     disp('What Standard Deviation for the noise?');
     standard_deviation = input('');

    %Obtain the mean from the user 
     disp('What is the Mean Value for the noise?');
     mean_data = input('');
     disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

     %Generate two sets of random data (GPS x random and y)
     random_values_pos = standard_deviation.* randn([length(total), 2]) + mean_data.* ones([length(total), 2]);
     if(new_reply == 2)
          disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
          fprintf('Range of "time" for data goes from 1 to %d.\n', length(total))
          while (min_input < 1 || min_input > length(total)-1)
              fprintf('Input minimum range for data set (must be number from 1 to %d) \n', length(total)-1)
              min_input = input('');
          end
          while (max_input > length(total) || max_input < min_input || max_input < 1 )
              fprintf('Input maximum range for data set (must be <= %d and >= min value) \n', length(total))
              max_input = input('');
          end
          disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
     end
end

%Get Odometry IMU and GPS data (x, y, theta)
Odom_x = data.O_x;
Odom_y = data.O_y;
Odom_theta = data.O_t;

%Obtain GPS position 
Gps_x = data.G_x;
Gps_y = data.G_y;
%If user does not choose 4 then keep orginally stored data 
if (reply ~= 4)
    disp('Normal storing for GPS data.')    
end
%If case 4 and full set requested, Add noise to GPS position
if (reply == 4 && new_reply == 1)    
    disp('Adding random noise to all x and y GPS data.');
    Gps_x = data.G_x + random_values_pos(:, 1);
    Gps_y = data.G_y + random_values_pos(:, 2);
end
%If case 4 and portion then use for loop to assign random noise to portion 
if (reply == 4 && new_reply == 2)   
    fprintf('Assigning random noise to range %d to %d for position of GPS. \n', min_input, max_input);
    for index=min_input:max_input
        Gps_x(index) = data.G_x(index) + random_values_pos(index, 1);
        Gps_y(index) = data.G_y(index) + random_values_pos(index, 2);
    end
end


%Get covariance data for GPS
Gps_Co_x = data.Co_gps_x;
Gps_Co_y = data.Co_gps_y;  
%Keep stored data if user choose 1 or 3 case 
if (reply == 1 || reply == 3) 
    disp('Normal storing for GPS Covariance data.')
end
%If case 2 full data set or 4 (either), modify covariance by random noise
if ((reply == 2 && new_reply == 1) || (reply == 4 && new_reply == 1))
    fprintf('Adding random noise to all Covariance of GPS\n');
    Gps_Co_x = data.Co_gps_x + random_values_pos(:, 1);
    Gps_Co_y = data.Co_gps_y + random_values_pos(:, 2);
end
%If case 2 add noise to covariance if porition requested 
if ((reply == 2 && new_reply == 2) || (reply == 4 && new_reply == 2)) 
    fprintf('Assigning random noise to range %d to %d for Covariance of GPS. \n', min_input, max_input);
    for index=min_input:max_input
        Gps_Co_x(index) = data.Co_gps_x(index) + random_values_pos(index, 1);
        Gps_Co_y(index) = data.Co_gps_y(index) + random_values_pos(index, 2);
    end
end

%Store IMU data 
IMU_heading = data.I_t;

%Store IMU covarience 
IMU_Co_heading = data.Co_I_t;

%If case 3 not chosen, keep stored data 
if(reply ~= 3)
     disp('Normal storing for IMU Covariance data.') 
end
%If case 3 full porition then add noise to all of data 
if (reply == 3 && new_reply == 1)
    fprintf('Adding random noise to all Covariance of IMU \n');
    IMU_Co_heading = data.Co_I_t + random_values_pos(:, 1);
end
if(reply == 3 && new_reply == 2)
    fprintf('Assigning random noise to range %d to %d for Covariance of IMU. \n', min_input, max_input);
    for index=min_input:max_input
        IMU_Co_heading(index) = data.Co_I_t(index) + random_values_pos(index, 1);
    end
end 

% Calibrate IMU to match with the robot's heading initially
IMU_heading = IMU_heading +(0.32981-0.237156)*ones(length(IMU_heading),1); 
%For EKF_DATA3

%Obtain user input on particular variables 
variable_input = 0; 
while(variable_input < 1 || variable_input > 2)
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    disp('Which P Q R and V would you like to use?')
    disp('  1. Use the Coding Insturction Docx Variables')
    disp('  2. Use the Lecture 9 Vairbles')
    variable_input = input('');
end
disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')


%********INITIALIZE STATES & OTHER INFORMATION***********%
%Depending on user input store different attributes 
if(variable_input == 1)
        %Velocity of the robot
    V = 0.14;%0.083;
     
    %Distance between 2 wheel
    L = 1; %meter
     
    %Angular Velocity
    Omega = V*tan(Odom_theta(1))/L;
     
    %set time_step
    delta_t = 0.001; %0.001
     
    s.x = [Odom_x(1); Odom_y(1); V; Odom_theta(1); Omega]; %Enter State (1x5)
     
    %Enter transistion matrix A (5x5)
    s.A = [1 0 delta_t*cos(Odom_theta(1)) 0 0;
           0 1 delta_t*sin(Odom_theta(1)) 0 0;
           0 0 1                          0 0;
           0 0 0                          1 delta_t;
           0 0 0                          0 1]; 
     
    %Define a process noise (stdev) of state: (Student can play with this number)
    %Enter covariance matrix Q (5x5) for state x
     
    s.Q = [.0004  0   0   0   0; %For EKF_DATA_circle
            0  .0004  0   0   0;
            0   0   .001  0   0;
            0   0    0  .001  0;
            0   0    0   0  .001]; 
     
    %   s.Q = [.000000004  0   0   0   0; %For EKF_DATA_Rutgers_ParkingLot
    %         0  .000000004  0   0   0;
    %         0   0   .001  0   0;
    %         0   0    0  .001  0;
    %         0   0    0   0  .001];  
    %Define the measurement matricx H:
    %Enter measurement matrix H (5x5) for measurement model z
    s.H = [ 1   0   0   0   0;
            0   1   0   0   0;
            0   0   1   0   0;
            0   0   0   1   0;
            0   0   0   0   1]; 
     
    %Define a measurement error (stdev)
    %Enter covariance matrix R (5x5) for measurement model z
    s.R = [.04  0   0   0   0;
            0  .04  0   0   0;
            0   0  .01  0   0;
            0   0   0   0.01  0;
            0   0   0   0  .01]; 
    %B matrix initialization:
    s.B = [ 1   0   0   0   0;
            0   1   0   0   0;
            0   0   1   0   0;
            0   0   0   1   0;
            0   0   0   0   1];
    %Enter initial value of u (5x5)    
    s.u = [0; 0; 0; 0; 0];
     
    %Enter initial covariance matrix P (5x5)
    s.P = [.001  0   0   0   0;
            0  .001  0   0   0;
            0   0  .001  0   0;
            0   0   0  .001  0;
            0   0   0   0  .001]; 
end
if(variable_input == 2)
    V = 0.44;%Velocity of the robot
    L = 1; %Distance between 2 wheel
    Omega = V*tan(Odom_theta(1))/L; %Angular Velocity
    delta_t = 0.001; %set time_step
    s.x = [Odom_x(1); Odom_y(1); V; Odom_theta(1); Omega]; %Enter State (1x5)
    s.Q = [.00001  0   0   0   0; %you can play with this matrix to see what happens
            0  .00001  0   0   0;
            0   0   .001  0   0;
            0   0    0  .001  0;
            0   0    0   0  .001]; %Enter covariance matrix Q (5x5) for state x
    s.H = [ 1   0   0   0   0;
            0   1   0   0   0;
            0   0   1   0   0;
            0   0   0   1   0;
            0   0   0   0   1]; %Enter measurement matrix H (5x5) for measurement model z
    s.R = [.1  0   0   0   0;
            0  .1  0   0   0;
            0   0  .01  0   0;
            0   0   0   0.01  0;
            0   0   0   0  .01]; %Enter covariance matrix R (5x5) for measurement model z
    s.B = [ 1   0   0   0   0;
            0   1   0   0   0;
            0   0   1   0   0;
            0   0   0   1   0;
            0   0   0   0   1];
    s.u = [0; 0; 0; 0; 0];
    s.P = [.01  0   0   0   0;
            0  .01  0   0   0;
            0   0  .01  0   0;
            0   0   0  .01  0;
            0   0   0   0  .01]; %Enter initial covariance matrix P (5x5)
end

%Begin Analyzing 
for t=1:length(total)
%Enter transistion matrix A (5x5)
 s(t).A = [1 0 delta_t*cos(Odom_theta(t)) 0 0;
       0 1 delta_t*sin(Odom_theta(t)) 0 0;
       0 0 1                          0 0;
       0 0 0                          1 delta_t;
       0 0 0                          0 1]; 
%Enter covariance matrix R (5x5) for measurement model z
 s(t).R = [Gps_Co_x(t)  0             0   0                    0;
            0            Gps_Co_y(t)   0   0                    0;
            0            0            .01  0                    0;
            0            0             0   IMU_Co_heading(t)    0;
            0            0             0   0                   .01]; 
%Enter measurement vector
 s(t).z =   [Gps_x(t); Gps_y(t); V; IMU_heading(t); Omega]; 

% perform a Kalman filter iteration (you need to implement it)
 s(t+1)=Kalman_Filter(s(t)); 
 
%Store Kalman filter x and y into their variables for plotting 
%Note that X matrix has [x, y, velocity, theta (orientation), accelration]
%Therefore x(1, :) is x and x(2, :) is y 
    x_position_kalman(t) = s(t).x(1,:);
    y_position_kalman(t) = s(t).x(2,:);
    t_theta_orientation_kalman(t) = s(t).x(4, :);
end

%Obtain user input concerning graphs 
graph_input = 0; 
while (graph_input < 1 || graph_input > 4)
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    disp('Which graphs would you like to see:')
    disp('  1. Position X Y ')
    disp('  2. Position Radian Iteration ')
    disp('  3. Both ')
    graph_input = input('');
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
end

if (graph_input == 1 || graph_input == 3)
    %Plot the position of the robot using various sets of data 
    if(graph_input == 3)
        subplot(1,2,1);
    end
    hold on
    plot(Odom_x, Odom_y, '.r')
    plot(Gps_x, Gps_y, '.g')
    plot(x_position_kalman, y_position_kalman, ".b" )
    %plot(Odom_x, Odom_y, 'r.',  Gps_x, Gps_y, 'r.',  x_position_kalman,  y_position_kalman, 'r.')
    grid on
    title('Fusion of GPS+IMU and ODOMETRY in Position')
    xlabel('X(m)')
    ylabel('Y(m)')
    xlim([-15 10])
    ylim([-5 20])
    legend('Odometry','GPS Calibrated', 'Kalman Output')
end

if (graph_input == 2 || graph_input == 3)
    %Plot the angular position of the robot using GPS+IMU
    if(graph_input  == 3)
        subplot(1,2,2);
    end
    plot(time, Odom_theta, "red", time, IMU_heading, "green",  time, t_theta_orientation_kalman, "blue", LineWidth=1.1)
    grid on
    title('Fusion of GPS+IMU and ODOMETRY in Position')
    xlabel('Iteration')
    ylabel('Radian')
    xlim([0 4000])
    ylim([-4 4])
    legend('Odometry Heading','IMU Heading', 'Kalman Filter Heading')
end

end
