% This file is made for testing your parameters. Change only the sections
% below where it is indicate. 
% Once the modifications are done, just run the file and see how the arm
% move with your parameters.
% If your DH parameters ar good you will see an
% animation of the arm following the same path as pre-registered.

Rad2Deg = 180/pi;

% We load the matrix where de joint position are stored
JntPos = load ('JntPos.mat');
Pee = load('Pee.mat');

% We assign each row to a joint
Q1 = Rad2Deg.*JntPos.ans(2,:);
Q2 = Rad2Deg.*JntPos.ans(3,:);
Q3 = Rad2Deg.*JntPos.ans(4,:);
Q4 = Rad2Deg.*JntPos.ans(5,:);
Q5 = Rad2Deg.*JntPos.ans(6,:);
Q6 = Rad2Deg.*JntPos.ans(7,:);

% Dimesions of a Jaco2 Spherical wrist
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;
    


for index = 1:length(JntPos.ans(1,:))
    
% ---------------------------------------------------------------------------------------------------------------
% ----------------------------- Make your change here to test DH parameters --------------------------------------
% ----------------------------------------------------------------------------------------------------------------
    % Choose your convention
    % Convention = 'Classic';
    Convention = 'Modified';
    % Add the right value to pass from your DH frames to the real robot
    % frames 
    % Exemple : q(1) = Q1(index) + 360;
    
    q(1) = Q1(index);
    q(2) = Q2(index);
    q(3) = Q3(index);
    q(4) = Q4(index);
    q(5) = Q5(index);
    q(6) = Q6(index);
    
    % Add your definition of the Trasformation Matrix between the world 
    % arm's frame and the first DH frames that you just created 
    T0 = [  0   0   0   0;...
            0   0   0   0;...
            0   0   0   0;...
            0   0   0   0];
   
    % Define your DH parameters in the folowing matrix. You must keep the same
    % syntax for the angles q and and use the dimensions of Jaco2 with a
    % spherical wrist just above.
    
    %       alpha   a       d       theta   
    DH = [  0,      0,      0,      q(1);...
            0,      0,      0,      q(2);...
            0,      0,      0,      q(3);...
            0,      0,      0,      q(4);...
            0,      0,      0,      q(5);...
            0,      0,      0,      q(6)];  
   
% ----------------------------------------------------------------------------------------------------------------   
% --------------------- Make the change here to test your foward kinematics --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

% The fisrt line call a ready to work function that you can use to test your
% DH parameters.
% The second line call a function that you must complete to make it works.

% Comment/Uncomment the line that correspond to your situation
%    coordinates = forwardKinematicsJaco6DOFS_complete(q,DH,T0,Convension);
   coordinates = forwardKinematicsJaco6DOFS_to_complete(q,DH,T0,Convension);

% ----------------------------------------------------------------------------------------------------------------   
% ------------------------------- Do not change the code a beyond this line --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

   % We store the position of the end effector in EndEffector
   EndEffector(index,:)=coordinates(:,5);
   
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
   xlim([-1 1])
   ylim([-1 1])
   zlim([0 1.3])
   
   % We mark the base with a red *
   plot3(coordinates(1,1),coordinates(2,1),coordinates(3,1), '-r*')
   
   % We show the trajectory for the whole simulation
   plot3(EndEffector(1:index,1),EndEffector(1:index,2),EndEffector(1:index,3),'.r')
   plot3(Pee.ans(2,:),Pee.ans(3,:),Pee.ans(4,:))
   hold off

end