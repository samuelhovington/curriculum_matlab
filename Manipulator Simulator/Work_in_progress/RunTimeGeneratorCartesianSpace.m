clear all;

theta = [-152; 189;72;186;200;0];

D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;
    
% Choose your convention
    convention = 'Classic';
%     convention = 'Modified';
    
% Choose the unit of the angles
    angleUnit = 'Degrees';
%     angleUnit = 'Radians';

% Enter you number of DOF
    DOF = 6;
    
% Add the right value to pass from your physical angles to your algorithm
% angles
% Exemple : q(1) = Q1(index) + 360;
    q(1,1) = theta(1,1)+180;
    q(2,1) = theta(2,1)+90;
    q(3,1) = theta(3,1)+90;
    q(4,1) = theta(4,1);
    q(5,1) = theta(5,1);
    q(6,1) = (theta(6,1)-90);
% Define your DH parameters in the folowing matrix. You must keep the same
% syntax for the angles q and and use the dimensions of Jaco2 with a
% spherical wrist just above.
%       alpha   a       d 
DH = [  pi/2,      0,      -D1;
        pi,  D2,      0;
        pi/2,     0,     -e2;
        pi/2,   0,      -(D3+D4);
        pi/2,   0,      0;
        pi,   0,      -(D5+D6)];

% Add your definition of the Trasformation Matrix between the world 
% arm's frame and the first DH frames that you just created 
    TW0=[1   0   0   0;
        0   -1  0   0;
        0   0   -1  0;
        0   0   0   1];

X_i = [0.35; -0.2652; 0.5062; 1.65; 1.1089; 0.1259];
X_g = [0.50; -0.2652; 0.5062; 1.65; 1.1089; 0.1259];
T = 2;

Cartesian_trajectory = TrajectoryPlanner_6DOFS_LBC_complete(X_i, X_g, T);
l = length(Cartesian_trajectory);
trajectory(1,:) = Cartesian_trajectory(1,:);
trajectory(2:7,1) = q;
trajectory(2:7,1) = InverseKinematics_complete(convention, DOF,DH, TW0, Cartesian_trajectory(2:7,1), trajectory(2:7,1), angleUnit);
for ii=2:l
    q = InverseKinematics_complete(convention, DOF,DH, TW0, Cartesian_trajectory(2:7,ii), trajectory(2:7,1), angleUnit);
    
    for index = 1:DOF
        while q(index)<=0||q(index)>=(2*pi)
           if q(index)<0
              q(index) = q(index)+2*pi;
           else
               q(index) = q(index)-2*pi;
           end
        end
    end
    trajectory(2:7,ii) = q;
end
angleUnit = 'Radians';


for index=1:l
    coordinates = forwardKinematicsJaco6DOFS_complete(convention,DH,TW0,trajectory(2:7,index),angleUnit)
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
%    for ii=1:DOF
%     plot3(Pgoal(1,1,ii),Pgoal(2,1,ii),Pgoal(3,1,ii), '-o')
%    end
   hold off
  
end


function theta = ThetaAlgo(theta)
Deg2Rad = pi/180;
theta(1) = Deg2Rad * (theta(1) + 180);
theta(2) = Deg2Rad * (theta(2) +90);
theta(3) = Deg2Rad * (theta(3) + 90);
theta(4) = Deg2Rad * theta(4);
theta(5) = Deg2Rad * theta(5);
theta(6) = Deg2Rad * (theta(6) - 90);
end