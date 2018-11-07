%Creation: 2018-10-18 by Simon Michaud @Kinova
%Modifications 2018-11-07

%Trajectory planner using a polynomial function in the joint space scheme

%Arguments: Initial angular position of the robot, goal angular position,
            %desired time
%Returns:   7x100*t(in sec) matrix that define the trajectory, with the 
            %first row being the time and the 6 other being the angular 
            %positions of the robot
            
function trajectory = TrajectoryPlanner_6DOFS_CP_to_complete(theta_i, theta_g, desired_time)
    JOINTS = 6;

    h = desired_time/(desired_time*100);

% ----------------------------------------------------------------------------------------------------------------
% ---------------------------------------- Enter your parameters here --------------------------------------------
% ----------------------------------------------------------------------------------------------------------------

    %Definition of the parameters going into the equations
    for i=1:JOINTS
       a0(i) = 0;
       a1(i) = 0;
       a2(i) = 0;
       a3(i) = 0;
    end

    %Generating the trajectory
    j= 1;
    %Adding the time in the first row of the trajectory matrix, DO NOT CHANGE
    for t=0+h:h:desired_time
       trajectory(1,j) = t;
       joint_velocity(1,j) = t;
       joint_acceleration(1,j) = t;

% ---------------------------------------------------------------------------------------------------------------
% ------- Insert the correct values of time and the correct trajectory equation for the different regions -------
% ---------------------------------------------------------------------------------------------------------------
       for i = 1:JOINTS
          trajectory(i+1,j) = 0;
          joint_velocity(i+1,j) = 0;
          joint_acceleration(i+1,j) = 0;
       end
       j= j+1;
    end

end