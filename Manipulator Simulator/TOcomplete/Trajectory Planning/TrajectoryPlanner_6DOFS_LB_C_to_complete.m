%Created on 2018-10-22 by Simon Michaud @Kinova
%Modified on 2018-11-06

%Trajectory planner using a linear function with blends in the cartesian
%space scheme

%Arguments: Initial cartesian position of the end effector, goal cartesian 
            %position of the end effecto desired time
%Returns:   7x100*t(in sec) matrix that define the trajectory, with the 
            %first row being the time and the 6 other being the cartesian
            %position and orientation of the end effector

function trajectory = TrajectoryPlanner_6DOFS_LB_C_to_complete(X_i, X_g, T)
    JOINTS = 6;
    % Definition of the time parameters
    ta = T/3;               %ta is the time in the acceleration blend
    h = T/(T*100);          %h is the time step for the trajectory matrix
    
    %Definition of the parameters of the algorithm
    L = X_g - X_i;          %Displacement
    
% ----------------------------------------------------------------------------------------------------------------
% ---------------------------------------- Enter your parameters here --------------------------------------------
% ----------------------------------------------------------------------------------------------------------------
    dX = 0;         %Velocity at the end of the first blend and the linear part
    ddX = 0;      %Acceleration in the blend
    
    % Parameters entered in the trajectory equation
    for i = 1: JOINTS
        a0(i) = 0;
        a1(i) = 0;
        a2(i) = 0;
        b0(i) = 0;
        b1(i) = 0;
        c0(i) = 0;
        c1(i) = 0;
        c2(i) = 0;
    end
    
    % Generation of the trajectory
    j= 1;
    
    %Adding the time in the first row of the trajectory matrix, DO NOT CHANGE
    for t=0+h:h:T
       trajectory(1,j) = t;
       joint_velocity(1,j) = t;
       joint_acceleration(1,j) = t;

% ---------------------------------------------------------------------------------------------------------------
% ------- Insert the correct values of time and the correct trajectory equation for the different regions -------
% ---------------------------------------------------------------------------------------------------------------
        for i = 1:JOINTS
            %Verification that the trajectory is in the acceleration phase
            if (t<0) && (t>=0)
              trajectory(i+1,j) = 0;
              joint_velocity(i+1,j) = 0;
              joint_acceleration(i+1,j) = 0;
            
            %Verification that the trajectory is in the linear phase
            elseif (t<0) && (t>=0)
              trajectory(i+1,j) = 0;
              joint_velocity(i+1,j) = 0;
              joint_acceleration(i+1,j) =0;
            
            %Verification that the trajectory is in the deceleration phase
            elseif (t<=0) && (t >=0)
              trajectory(i+1,j) = 0;
              joint_velocity(i+1,j) = 0;
              joint_acceleration(i+1,j) = 0; 
            end
        end
       j = j+1; 
    end

end