close all; clear all;
%File that runs the verification of the Inverse Kinematics function. Change indicated
%lines to make the script work correctly.

Pgoal(:,:,1) = [0.2114; -0.2652; 0.5062; 1.6499; 1.1089; 0.1259];
Pgoal(:,:,2) = [0.23; -0.2652; 0.5062; 1.85; 1.1089; 0.1259];
Pgoal(:,:,3) = [0.23; -0.2452; 0.5062; 1.85; 1.11; 0.1259];
Pgoal(:,:,4) = [0.23; -0.2452; 0.49; 1.85; 1.11; 0.1259];
Pgoal(:,:,5) = [0.23; -0.2452; 0.49; 2; 1.1;-1.55];
Pgoal(:,:,6) = [0.22; -0.25; 0.50; 1.90; 1.1;-1.55];
Pgoal(:,:,7) = [0.2114; -0.2652; 0.5062; 1.6499; 1.1089; 0.1259];

theta(:,1) = [-76,163,44,266,257,0];
Rad2Deg = 180/pi;

% Dimensions of a Jaco2 Spherical wrist
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;
    

% ---------------------------------------------------------------------------------------------------------------
% -------------------- Change the different parts of this section for your situation ----------------------------
% ---------------------------------------------------------------------------------------------------------------
% Choose your convention
%     Convention = 'Classic';
    Convention = 'Modified';
    
% Choose the unit of the angles
    AngleUnit = 'Degrees';
%     AngleUnit = 'Radians';

% Enter you number of DOF
    DOF = 6;
    
% Add the right value to pass from your physical angles to your algorithm
% angles
% Exemple : q(1) = Q1(index) + 360;
    q(1,1) = theta(1,1)+180;
    q(2,1) = theta(2,1)+90;
    q(3,1) = theta(3,1)+90;
    q(4,1) = -theta(4,1);
    q(5,1) = -theta(5,1);
    q(6,1) = -(theta(6,1)-90);
% Define your DH parameters in the folowing matrix. You must keep the same
% syntax for the angles q and and use the dimensions of Jaco2 with a
% spherical wrist just above.
%       alpha   a       d 
DH = [  0,      0,      -D1;
        pi/2,   0,      0;
        pi,     D2,     -e2;
        3*pi/2,   0,    (D3+D4);
        pi/2,   0,      0;
        pi/2,   0,     (D5+D6)];

% Add your definition of the Trasformation Matrix between the world 
% arm's frame and the first DH frames that you just created 
    T0=[1   0   0   0;
        0   -1  0   0;
        0   0   -1  0;
        0   0   0   1];
    
% ----------------------------------------------------------------------------------------------------------------   
% ------------------------------ Make the change here to test your Jacobian --------------------------------------
% ---------------------------------------------------------------------------------------------------------------- 

q(:,1) = InverseKinematics_complete(Convention,DOF,DH,T0,Pgoal(:,:,1), q(:,1), AngleUnit);
% q(:,1) = InverseKinematics_to_complete(Convention,DOF,DH,T0,Pgoal(:,:,1), q(:,1), AngleUnit);
for ii =2: 7
    q(:,ii) = InverseKinematics_complete(Convention,DOF,DH,T0,Pgoal(:,:,ii), q(:,ii-1), AngleUnit);
%     q(:,ii) = InverseKinematics_to_complete(Convention,DOF,DH,T0,Pgoal(:,:,ii), q(:,ii-1), AngleUnit);
end
AngleUnit = 'Radians';

% % ----------------------------------------------------------------------------------------------------------------   
% % ------------------------------- Do not change the code beyond this line --------------------------------------
% % ----------------------------------------------------------------------------------------------------------------
for index=1:7
    coordinates = forwardKinematicsJaco6DOFS_complete(Convention,DH,T0,q(:,index),AngleUnit)
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
   
   % We mark the base with a red *
   plot3(coordinates(1,1),coordinates(2,1),coordinates(3,1), '-r*')
   
   % We show the trajectory for the whole simulation
   plot3(EndEffector(1:index,1),EndEffector(1:index,2),EndEffector(1:index,3),'.r')
   for ii=1:DOF
    plot3(Pgoal(1,1,ii),Pgoal(2,1,ii),Pgoal(3,1,ii), '-o')
   end
   hold off
   pause
end