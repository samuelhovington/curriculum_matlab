close all; clc; clear all;
%File that runs the verification of the DH parameters and the foward
%kinematics functions. Change indicated lines to make the script work
%correctly.

% ----------------------------------------------------------------------------------------------------------------
% -------------------------------------------- Change your path --------------------------------------------------
% ----------------------------------------------------------------------------------------------------------------

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

% Dimensions of a Jaco2 Spherical wrist
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;
     
txt2 = 'If everything is correct, your robot (in blue) should follow the trajectory (in orange).';
txt1 = 'The goal of this exercise is to verify your DH parameters and your forward kinematics function by verifying that the robot executes the correct trajectory.';
txt3 = '';
txt4 = 'When you are done reading, press ok to start the program.';
uiwait(msgbox({txt1 txt2 txt3 txt4}, 'Verification DH FK'));

for index = 1:length(JntPos.ans(1,:))
    
% ---------------------------------------------------------------------------------------------------------------
% ----------------------------- Make your change here to test DH parameters --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

% Choose your convention
    Convention = 'Classic';
%     Convention = 'Modified';
    
% Choose the unit of the angles
    AngleUnit = 'Degrees';
%     AngleUnit = 'Radians';

% Add the right value to pass from your physical angles to your algorithm
% angles
% Exemple : q(1) = Q1(index) + 360;
    
    q(1) = Q1(index)+180;
    q(2) = Q2(index)+90;
    q(3) = Q3(index)+90;
    q(4) = Q4(index);
    q(5) = Q5(index);
    q(6) = (Q6(index)-90);
    
    
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
    DH = [  pi/2,   0,      -D1,        q(1);
            pi,     D2,     0,          q(2);
            pi/2 ,  0,      -e2,        q(3);
            pi/2,   0,      -(D3+D4),   q(4);
            pi/2,   0,      0,          q(5);
            pi,     0,      -(D5+D6),   q(6)];
   

% ----------------------------------------------------------------------------------------------------------------   
% --------------------- Make the change here to test your foward kinematics --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

% The fisrt line call a ready to work function that you can use to test your
% DH parameters.
% The second line call a function that you must complete to make it works.

% Comment/Uncomment the line that correspond to your situation

   coordinates = forwardKinematicsJaco6DOFS_complete(Convention,DH,T0,q,AngleUnit);
%    coordinates = forwardKinematicsJaco6DOFS_to_complete(Convention,DH,T0,q);

% ----------------------------------------------------------------------------------------------------------------   
% ------------------------------- Do not change the code beyond this line --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

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
   title('Verification of the DH Parameters and the Foward Kinematics Function')
%    xlim([-1 1])
%    ylim([-1 1])
%    zlim([0 1.3])
   
   % We mark the base with a red *
   plot3(coordinates(1,1),coordinates(2,1),coordinates(3,1), '-r*')
   
   % We show the trajectory for the whole simulation
   plot3(EndEffector(1:index,1),EndEffector(1:index,2),EndEffector(1:index,3),'.r')
   plot3(Pee.ans(2,:),Pee.ans(3,:),Pee.ans(4,:))
   hold off

end