clear all;
Q1 = 180;
Q2 = 270;
Q3= 180;
Q4 = 270;
Q5 = 270;
Q6 = 270;



D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;

% Choose your convention
%     Convention = 'Classic';
    Convention = 'Modified';
    
% Choose the unit of the angles
    AngleUnit = 'Degrees';
%     AngleUnit = 'Radians';

% Add the right value to pass from your physical angles to your algorithm
% angles
% Exemple : q(1) = Q1(index) + 360;
    
    q(1) = Q1+180;
    q(2) = Q2;
    q(3) = Q3;
    q(4) = Q4;
    q(5) = Q5;
    q(6) = Q6;
    
    
% Add your definition of the Trasformation Matrix between the world 
% arm's frame and the first DH frames that you just created 
    T0=[1   0   0   0;
        0   -1  0   0;
        0   0   -1  0;
        0   0   0   1];

% Define your DH parameters in the folowing matrix. You must keep the same
% syntax for the angles q and and use the dimensions of Jaco2 with a
% spherical wrist just above.
    
    %       alpha   a       d           theta   
    DH = [  0,   0,      -D1,        q(1);
            pi/2,     0,     0,          q(2);
            pi,  D2,      -e2,        q(3);
            3*pi/2,   0,      (D3+D4),   q(4);
            pi/2,   0,      0,          q(5);
            3*pi/2,     0,      -(D5+D6),   q(6)];
   % The fisrt line call a ready to work function that you can use to test your
% DH parameters.
% The second line call a function that you must complete to make it works.

% Comment/Uncomment the line that correspond to your situation

   coordinates = forwardKinematicsJaco6DOFS_complete(Convention,DH,T0,q,AngleUnit);
%    coordinates = forwardKinematicsJaco6DOFS_to_complete(Convention,DH,T0,q);
    orientation(:,1) = coordinates(4:6, 7);
% ----------------------------------------------------------------------------------------------------------------   
% ------------------------------- Do not change the code beyond this line --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

   % We store the position of the end effector in EndEffector
   EndEffector(1,:)=coordinates(:,7);
   
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
   plot3(EndEffector(1,1),EndEffector(1,2),EndEffector(1,3),'.r')
   plot3(Pee.ans(2,:),Pee.ans(3,:),Pee.ans(4,:))
   hold off


x = load('orientation2.mat')
d = x.orientation - orientation

if (max(d) < 0.00001)
   1
end