%Gabriel Mortensen
%Mar. 2022 
%CPE 470 
%Project 2

function Project_2()

% *** Initialization for clearing all input *** 
clc, close all
clear 

% *** Obtain User Input *** 
reply = 0; 
while (reply < 1 || reply > 6  )
    disp('!!!Gabriel Mortensen Project 2 CPE 470!!!')
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    disp('What would you like to do?')
    disp('  1: Noise free environment circular trajectory.')
    disp('  2: Noisy environment circular trajectory.')
    disp('  3: Noise free environment linear/line trajectory.')
    disp('  4: Noisy environment linear/line trajectory.')
    disp('  5: Noise free environment sin trajectory.')
    disp('  6: Noisy environment sin trajectory.')
    reply = input('');
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
end

% *** Set parameters for simulation ***
n = 2; % Number of dimensions
delta_t = 0.05; % Set time step
t = 0:delta_t:10;% Set total simulation time
lambda = 8.5; % Set scaling factor of attractive potential field
vr_max = 50; % Set maximum of robot velocity
Phi =  zeros (length(t),1); % Inital Phi of the robot 

% *** Set VIRTUAL TARGET ***
qv = zeros (length(t),n); %Initial positions of virtual target
pv = 1.2; %Set velocity of virtual target
theta_t = zeros (length(t),1); % Initial heading of the virtual target

% *** Set ROBOT ***
%Set initial state of robot (robot)
qr = zeros (length(t),n); %initial position of robot
v_rd =  zeros (length(t),1); %Initial velocity of robot
theta_r = zeros (length(t),1); % Initial heading of the robot

% *** Obtain user input again on robot velocity ***
new_reply = 0; 
while (new_reply < 1 || new_reply > 2  )
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    fprintf("The current velocity of the VIRTUAL TARGET is: %.2f \n", pv)
    fprintf("The current limit on the velocity of the ROBOT is: %d \n", vr_max)
    disp('Do you want to change the limit on the ROBOT velocity?')
    disp('  1: Yes.')
    disp('  2: No.')
    new_reply = input('');
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
end
% *** react to user input if required ***
if(new_reply == 1)
      disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
      disp('What speed limit would you like to place on the robot?')
      vr_max = input('');
      disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
end 

% *** Obtain user input again on robot velocity ***
extra_reply = 0; 
while (extra_reply < 1 || extra_reply > 2  )
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    fprintf("The current lambda is: %.2f \n", lambda)
    disp('Do you want to change the lambda?')
    disp('  1: Yes.')
    disp('  2: No.')
    extra_reply = input('');
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
end
% *** react to user input if required ***
if(extra_reply == 1)
      disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
      disp("What would you like lambda to be?")
      lambda = input('');
      disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
end 

% *** Set relative states between ROBOT and VIRTUAL TARGET ***
qrv = zeros (length(t),n); %Save relative positions between robot and virtual target
prv = zeros(length(t),n); %Save relative velocities between robot and virtual target

% *** Compute initial relative states between robot and  VIRTUAL TARGET ***
qrv(1,:) = qv(1,:) - qr(1,:);%Compute the initial relative position
prv(1,:) = [pv*cos(theta_t(1))-v_rd(1)*cos(theta_r(1)), pv*sin(theta_t(1))-v_rd(1)*sin(theta_r(1))]; %Compute the initial relative velocity

% *** Set noise mean and standard deviation ***
noise_mean = 0.5;
noise_std = 0.5; %try 0.2 also

% *** Get user input ***
if (mod(reply, 2) == 0)
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    disp('Please select the mean (typical is 0.5):')
    noise_mean = input('');
    disp('Please select the standard deviation (typical is 0.5 or 0.2):')
    noise_std = input('');
    disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
end

% *** Make a figure and maxmize it for user experience ***
Fig = figure();
Fig.WindowState = 'maximized';


%=========MAIN PROGRAM==================
for i =2:length(t)  
 
    %Store variable for previous state 
    Previous_State = i -1; 

    %++++++++++CIRCULAR TRAJECTORY+++++++++++
    if (reply == 1)
    %Set target trajectory moving in CIRCULAR trajectory WITHOUT noise
        qv_x = 60 - 15*cos(t(i));
        qv_y = 30 + 15*sin(t(i));
        qv(i,:) = [qv_x, qv_y]; %compute position of virtual target
    end
    if (reply == 2)
    %Set target trajectory moving in CIRCULAR trajectory WITH noise
        qv_x = 60 - 15*cos(t(i))+ noise_std * randn + noise_mean;
        qv_y = 30 + 15*sin(t(i)) + noise_std * randn + noise_mean;
        qv(i,:) = [qv_x, qv_y];  %compute position of target
    end
    if (reply == 3)
    %Set target trajectory moving in linear trajectory WITHOUT noise
        qv_x = i*2;
        qv_y = i*2;
        qv(i,:) = [qv_x, qv_y];  %compute position of target
    end
    if (reply == 4)
    %Set target trajectory moving in linear trajectory WITHOUT noise
        qv_x = i*2 + noise_std * randn + noise_mean;
        qv_y = i*2 + noise_std * randn + noise_mean;
        qv(i,:) = [qv_x, qv_y];  %compute position of target
    end
    if (reply == 5)
    %Set target trajectory moving in linear trajectory WITHOUT noise
        qv_x = 30 + t(i) * 7;
        qv_y = 30 + sin(t(i)) * 7;
        qv(i,:) = [qv_x, qv_y];  %compute position of target
    end
    if (reply == 6)
    %Set target trajectory moving in linear trajectory WITHOUT noise
        qv_x = 30 + 7 * t(i) + noise_std * randn + noise_mean;
        qv_y = 30 + 7 * sin(t(i)) + noise_std * randn + noise_mean;
        qv(i,:) = [qv_x, qv_y];  %compute position of target
    end
    %Compute the target heading
    qt_diff(i,:) =  qv(i,:)- qv(Previous_State,:);
    theta_t(i) = atan2(qt_diff(i,2),qt_diff(i,1)); 

    %======UPDATE position and velocity of robot==========
    %Phi calculation: Slide 2 from Project2_Instruction.ppt   
    Phi(i) = atan2(qrv(Previous_State,2), qrv(Previous_State,1)); 
    
    %v_rd calculation: Slide 4 from Project2_Instruction.ppt
    v_rd_expression = (norm(pv)^2) + (2*lambda*norm(qrv(Previous_State, :))*norm(pv)*abs(cos(theta_t(i)-Phi(i)))) + ((norm(qrv(Previous_State, :)))^2*(lambda)^2);
    v_rd(i) = sqrt(v_rd_expression);
   
    %Limit Robot Velocity if need be 
    v_rd(i) = min(v_rd(i), vr_max);

    %theta_r calculation: Slide 4 from Project2_Instruction.ppt
    %Calculates orientation (heading) 
    theta_r_expression_numerator = (norm(pv) * sin(theta_t(i) - Phi(i)));
    theta_r_expression_denominator = norm(v_rd(i));
    theta_r_expression = theta_r_expression_numerator / theta_r_expression_denominator; 
    theta_r(i) = Phi(i) + asin(theta_r_expression);
 
    %Calculate position of the robot 
    qr(i,:) = qr(Previous_State,:) + v_rd(i)*delta_t*[cos(theta_r(Previous_State)), sin(theta_r(Previous_State))];
    qrv(i,:) = qv(i,:) - qr(i,:);
    prv(i,:) = [pv*cos(theta_t(i)) - v_rd(i)*cos(theta_r(i)), pv*sin(theta_t(i)) - v_rd(i)*sin(theta_r(i))];

    error(i) = norm(qv(i,:)-qr(i,:));

    %plot postions qv of virtual target
    subplot(2,2,1);
    plot(qv(:,1),qv(:,2),'r>')
    hold on
    %plot postions qv of robot
    plot(qr(:,1),qr(:,2),'g>')
   
    M = getframe(gca); 
end

%label graph 
subplot(2,2,1);
grid on
xlabel("X Position")
ylabel("Y Position")
title('Robot Chasing Virtual Target')
legend('Virtual Target', 'Robot')

subplot(2,2,2);
plot(error(2:length(t)), 'b.')
grid on
title('Distance of Error')
xlabel("Iterations")
ylabel("Error")
legend('Distance error between robot and virtual target')

subplot(2,2,3);
plot(v_rd, 'b')
grid on
legend('Robot velocity')
xlabel("Iterations")
ylabel("Velocity")
title('Robot Velocity Over Iteration')

set(0,'CurrentFigure', Fig)
subplot(2,2,4);
hold on 
plot(theta_r, '--b') 
plot(theta_t, '-.r')
plot(Phi, 'k')
grid on
xlabel("Iterations")
ylabel("Radian")
title('Heading/Orientation Comparison Iterations')
legend('Robot orientation', 'Target orientation', 'Relative Orientation')

end
