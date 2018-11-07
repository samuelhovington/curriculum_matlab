close all; clear all;
%File that runs the verification of the cartesian trajectory planner 
%function. Change indicated lines to make the script work correctly.

%Parameters necessary to verify the function
CT = load('CT.mat');
CTPC = load('CTPC.mat');
X_i = [0.2114; -0.2652; 0.5062; 1.65; 1.1089; 0.1259];
X_g = [0.5; -0.5; 0.2562; 2; 1.5; 0.1259];
theta(:,1) = [283.23,162.57,43.44,-94.35,257.41,287.97];
Rad2Deg = 180/pi;
T = 2;

%Lengths of the robot
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
    convention = 'Classic';
%     Convention = 'Modified';

% Choose the unit of the angles
    angleUnit = 'Degrees';
%     AngleUnit = 'Radians';

% Enter you number of DOF
    DOF = 6;

% Add the right value to pass from your physical angles to your algorithm
% angles
% Exemple : q(1) = Q1(index) + 360;

    q(1) = theta(1);
    q(2) = theta(2);
    q(3) = theta(3);
    q(4) = theta(4);
    q(5) = theta(5);
    q(6) = theta(6);


% Add your definition of the Trasformation Matrix between the world 
% arm's frame and the first DH frames that you just created 
    TW0=[   1   0   0   0;
            0   1   0   0;
            0   0   1   0;
            0   0   0   1];

% Define your DH parameters in the folowing matrix. You must keep the same
% syntax for the angles q and and use the dimensions of Jaco2 with a
% spherical wrist just above.

    %       alpha   a       d           theta   
    DH = [  0,      0,      0,          q(1);
            0,      0,      0,          q(2);
            0,      0,      0,          q(3);
            0,      0,      0,          q(4);
            0,      0,      0,          q(5);
            0,      0,      0,          q(6)];

% ----------------------------------------------------------------------------------------------------------------   
% ------------------------------ Make the change here to test your trajectory planner --------------------------------------
% ---------------------------------------------------------------------------------------------------------------- 
    Cartesian_trajectory = TrajectoryPlanner_6DOFS_LB_C_complete(X_i, X_g, T);
    % Cartesian_trajectory = TrajectoryPlanner_6DOFS_LB_C_to_complete(X_i, X_g, T);

% % ----------------------------------------------------------------------------------------------------------------   
% % ------------------------------- Do not change the code beyond this line --------------------------------------
% % ----------------------------------------------------------------------------------------------------------------

    l = length(Cartesian_trajectory);                           %Finding the number of iterations necessary
    trajectory(1,:) = Cartesian_trajectory(1,:);                %Entering the time in the fist row of the trajectory
    trajectory(2:7,1) = q;
%Entering the first angles in the positions 2 to 7 of the first column of the trajectory matrix
    trajectory(2:7,1) = InverseKinematics_complete(convention, DOF,DH, TW0, Cartesian_trajectory(2:7,1), trajectory(2:7,1), angleUnit);
    angleUnit = 'Radians';                                      %The angles are now in radians
    
    for ii=2:l                                                  %Generating the trajectory
%Getting the angles from the cartesian position
        q = InverseKinematics_complete(convention, DOF,DH, TW0, Cartesian_trajectory(2:7,ii), trajectory(2:7,ii-1), angleUnit);

%Getting angles back in the 0 to 2pi range
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

    
txt2 = 'If everything is correct, your robot (in blue) should follow the path (in orange) and the correct trajectory (green robot).';
txt1 = 'The goal of this exercise is to verify your cartesian space linear motion function by observing if your robot executes the same trajectory.';
txt3 = '';
txt4 = 'When you are done reading, press ok to start the program.';
uiwait(msgbox({txt1 txt2 txt3 txt4}, 'Cartesian Planner'));
    
%Plotting the results
    for index=1:l
        coordinates = forwardKinematicsJaco6DOFS_complete(convention,DH,TW0,trajectory(2:7,index),angleUnit);
         % We store the position of the end effector in EndEffector
       EndEffector(index,:)=coordinates(:,7);

       % We create the figure where the robot will be shown
       figure(1)

       % We plot the position of each articulation
       c = plot3(coordinates(1,:),coordinates(2,:),coordinates(3,:));
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
       
       %Plotting the green robot that holds the correct trajectory
       p = plot3(CTPC.coordinates(1,:,index),CTPC.coordinates(2,:,index),CTPC.coordinates(3,:,index),'-g');
       p.LineWidth= 2;
       
       % We show the trajectory for the whole simulation
       plot3(EndEffector(1:index,1),EndEffector(1:index,2),EndEffector(1:index,3),'.r')
       plot3(CT.Cartesian_trajectory(2,:), CT.Cartesian_trajectory(3,:),CT.Cartesian_trajectory(4,:))
       hold off

    end