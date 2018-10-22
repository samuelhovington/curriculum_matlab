close all; clc;
addpath(genpath('C:\Users\shovington\Documents\GitHub\curriculum_matlab'))
cd 'C:\Users\shovington\Documents\GitHub\curriculum_matlab\Manipulator Simulator'

Rad2Deg = 180/pi;

% We load the matrix where de joint and the end effector position are stored
JntPos = load ('JntPos.mat');
Pee = load('Pee.mat');

% We assign each row to a joint
Q1 = Rad2Deg.*JntPos.ans(2,:);
Q2 = Rad2Deg.*JntPos.ans(3,:);
Q3 = Rad2Deg.*JntPos.ans(4,:);
Q4 = Rad2Deg.*JntPos.ans(5,:);
Q5 = Rad2Deg.*JntPos.ans(6,:);
Q6 = Rad2Deg.*JntPos.ans(7,:);


% Robot geometric parameters
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;

for index = 1:length(JntPos.ans(1,:))
%   Difference between physical angles Q and Dh algorithm angles q
    q(1) = Q1(index);
    q(2) = Q2(index)-90;
    q(3) = Q3(index)+90;
    q(4) = -Q4(index);
    q(5) = -Q5(index);
    q(6) = Q6(index);
    
   
%   Transformation matrices from the base to the joint i
    T0=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];

%   DH parameters
    DH = [0, 0, -D1, q(1); -pi/2, 0, 0, q(2); pi ,D2, -e2, q(3); 3*pi/2, 0, (D3+D4), q(4); pi/2, 0, 0, q(5); 3*pi/2, 0, -(D5+D6), q(6)];
    
%   Uncomment the convension of DH parameters you use

%   Convension = 'Classic';
   Convension = 'Modified';

%   We call the function that takes angular positions and return
%   cartesian position of each articulation. 
    coordinates = forwardKinematicsJaco6DOFS_for_verification(q,DH,T0,Convension); 
   
%   We store the position of the end effector in EndEffector
    EndEffector(index,:)=coordinates(:,5);
   
%   We create the figure where the robot will be shown
    figure(1)
   
%   We plot the position of each articulation
    plot3(coordinates(1,:),coordinates(2,:),coordinates(3,:))
   
    hold on
   
%   We set the display settings
    view(150,40) 
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
    axis equal
    grid on
    set(findall(gca, 'Type', 'Line'),'LineWidth',5);
    xlabel('X')
    ylabel('Y')
    xlim([-1 1])
    ylim([-1 1])
    zlim([0 1.3])
   
%   We mark the base with a red *
    plot3(coordinates(1,1),coordinates(2,1),coordinates(3,1), '-r*')
   
%   We show the trajectory for the whole simulation
    plot3(EndEffector(1:index,1),EndEffector(1:index,2),EndEffector(1:index,3),'.r')
    plot3(Pee.ans(2,:),Pee.ans(3,:),Pee.ans(4,:))
    hold off

end