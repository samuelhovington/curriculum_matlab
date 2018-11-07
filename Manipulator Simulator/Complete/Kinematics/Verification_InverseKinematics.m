close all; clear all;
%File that runs the verification of the Inverse Kinematics function. Change indicated
%lines to make the script work correctly.

C = load('CoordinatesIKV.mat');

Pgoal(:,:,1) = [0.2114; -0.2652; 0.5062; 1.64; 1.1089; 0.1259];
Pgoal(:,:,2) = [0.55; -0.2652; 0.5062; 1.85; 1.1089; 0.1259];
Pgoal(:,:,3) = [0.55; 0; 0.5062; 1.85; 1.55; 0.1259];
Pgoal(:,:,4) = [0.55; 0; 0.25; -2.3188; 1.55; -1.55];
Pgoal(:,:,5) = [0.55; -0.25; 0.25; -2.3188; 1.1; -1.55];
Pgoal(:,:,6) = [0.3; -0.25; 0.50; -2.3188; 1.11;-1.55];
Pgoal(:,:,7) = [0.2114; -0.2652; 0.5062; 1.64; 1.1089; 0.1259];

theta(:,1) = [283.23,162.57,43.44,-94.35,257.41,287.97];
Rad2Deg = 180/pi;

% ---------------------------------------------------------------------------------------------------------------
% -------------------- Change the different parts of this section for your situation ----------------------------
% ---------------------------------------------------------------------------------------------------------------
% Choose your convention
    Convention = 'Classic';
%     Convention = 'Modified';
    
% Choose the unit of the angles
    AngleUnit = 'Degrees';
%     AngleUnit = 'Radians';

% Enter you number of DOF
    DOF = 6;
    
% Add the right value to pass from your physical angles to your algorithm
% angles
% Exemple : q(1) = Q1(index) + 360;
    q(:,1) = Theta_physalgo(Convention, AngleUnit, 1,1,theta);
% Define your DH parameters in the folowing matrix. You must keep the same
% syntax for the angles q and and use the dimensions of Jaco2 with a
% spherical wrist just above.
%       alpha   a       d 
DH = DH(Convention, 1);
% Add your definition of the Trasformation Matrix between the world 
% arm's frame and the first DH frames that you just created 
    T0=[1   0   0   0;
        0   -1  0   0;
        0   0   -1  0;
        0   0   0   1];
    
% ----------------------------------------------------------------------------------------------------------------   
% ------------------------------ Make the change here to test your Inverse Kinematics --------------------------------------
% ---------------------------------------------------------------------------------------------------------------- 

q(:,1) = InverseKinematics_complete(Convention,DOF,DH,T0,Pgoal(:,:,1), q(:,1), AngleUnit);
% q(:,1) = InverseKinematics_to_complete(Convention,DOF,DH,T0,Pgoal(:,:,1), q(:,1), AngleUnit);
AngleUnit = 'Radians';
for ii =2: 7
    q(:,ii) = InverseKinematics_complete(Convention,DOF,DH,T0,Pgoal(:,:,ii), q(:,ii-1), AngleUnit);
%     q(:,ii) = InverseKinematics_to_complete(Convention,DOF,DH,T0,Pgoal(:,:,ii), q(:,ii-1), AngleUnit);
end

% % ----------------------------------------------------------------------------------------------------------------   
% % ------------------------------- Do not change the code beyond this line --------------------------------------
% % ----------------------------------------------------------------------------------------------------------------
txt2 = 'Your robot (in blue) should follow the lines representing the correct robot configuration (in green) and reach the circles of color with its end effector';
txt1 = 'The goal of this exercise is to verify your inverse kinematics function by verifying that it made the robot go to the desired cartesian positions in the correct configuration.';
txt3 = '';
txt4 = 'When you are done reading, press ok to start the program. To change position of verification, press any key.';
uiwait(msgbox({txt1 txt2 txt3 txt4}, 'Inverse Kinematics'));

for index=1:7
    coordinates = forwardKinematicsJaco6DOFS_complete(Convention,DH,T0,q(:,index),AngleUnit);
     % We store the position of the end effector in EndEffector
   EndEffector(index,:)=coordinates(:,7);
   
   % We create the figure where the robot will be shown
   figure(1)
   
   % We plot the position of each articulation
   plot3(coordinates(1,:),coordinates(2,:),coordinates(3,:))
   
   hold on
   
   % We set the display settings
   view(150,40) 
   set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
   axis equal
   grid on
   set(findall(gca, 'Type', 'Line'),'LineWidth',5);
   xlabel('X')
   ylabel('Y')
%    xlim([-1 1])
%    ylim([-1 1])
%    zlim([0 1.3])
%    
   % We mark the base with a red *
   plot3(coordinates(1,1),coordinates(2,1),coordinates(3,1), '-r*')
   
   % We show the trajectory for the whole simulation
   plot3(EndEffector(1:index,1),EndEffector(1:index,2),EndEffector(1:index,3),'.r')
   p = plot3(C.coordinates(1,:,index),C.coordinates(2,:,index),C.coordinates(3,:,index), '-g');
   p.LineWidth = 2;
   
   for ii=1:DOF
    plot3(Pgoal(1,1,ii),Pgoal(2,1,ii),Pgoal(3,1,ii), '-o')
   end
   hold off
   pause
end