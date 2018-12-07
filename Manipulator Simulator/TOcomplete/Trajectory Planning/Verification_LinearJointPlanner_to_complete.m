clear all; close all; clc;
Rad2Deg = 180/pi;
%File that runs the verification of the linear trajectory planner function 
%for the joint space scheme. Change indicated lines to make the script work
%correctly.

%Parameters necessary to verify the function
theta_i = [180,180,180,180,180,180];
theta_g = [90,270,90,270,270,270];
theta_v = [180,270,90,270,270,270];
desired_time = 1;

% ----------------------------------------------------------------------------------------------------------------   
% ------------------------- Make the change here to test your trajectory planner ---------------------------------
% ---------------------------------------------------------------------------------------------------------------- 

% JntPos = TrajectoryPlanner_6DOFS_LB_J_complete(theta_i, theta_g, desired_time);
JntPos = TrajectoryPlanner_6DOFS_LB_J_to_complete(theta_i, theta_g, desired_time);

% % ----------------------------------------------------------------------------------------------------------------   
% % ------------------------------- Do not change the code beyond this line --------------------------------------
% % ----------------------------------------------------------------------------------------------------------------
Q1 = JntPos(2,:);
Q2 = JntPos(3,:);
Q3 = JntPos(4,:);
Q4 = JntPos(5,:);
Q5 = JntPos(6,:);
Q6 = JntPos(7,:);

C = load('JLT.mat');

txt2 = 'If everything is correct, your robot (in blue) should follow the verification robot (in green).';
txt1 = 'The goal of this exercise is to verify your path and your trajectory by making sure that your robot follows our robot.';
txt3 = '';
txt4 = 'When you are done reading, press ok to start the program.';
uiwait(msgbox({txt1 txt2 txt3 txt4}, 'Linear Joint Planner'));


for index = 1:length(JntPos(1,:))
   coordinates = forwardKinematicsJaco6DOFS(Q1(index), Q2(index), Q3(index), Q4(index), Q5(index), Q6(index)); 
   EndEffector(index,:)=coordinates(:,5);
   
   figure(1);

   plot3(coordinates(1,:),coordinates(2,:),coordinates(3,:))
   hold on
   view(150,40) 
   set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
%    set ( gca, 'xdir', 'reverse' )
%     set ( gca, 'ydir', 'reverse' )
   plot3(coordinates(1,1),coordinates(2,1),coordinates(3,1), '-r*')
   axis equal
   set(findall(gca, 'Type', 'Line'),'LineWidth',5);
   xlabel('X')
   ylabel('Y')
   xlim([-1 1])
   ylim([-1 1])
   zlim([-1.3 1.3])
   plot3(EndEffector(:,1),EndEffector(:,2),EndEffector(:,3))
   
    %Plotting the green robot that holds the correct trajectory
       p = plot3(C.coordinates(1,:,index),C.coordinates(2,:,index),C.coordinates(3,:,index),'-g');
       p.LineWidth= 2;
   grid on
   hold off
     
end