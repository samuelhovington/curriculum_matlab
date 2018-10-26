%Function that receives two angles and that generates a trajectory to
%generate the movement between those two points

function trajectory =TrajectoryPlanner_6DOFS_LB_to_complete(theta_i, theta_g, T)
    JOINTS = 6;
    % Definition of the time parameters
    ta = T/3;               %ta is the time in the acceleration blend
    h = T/(T*100);          %h is the time step for the trajectory matrix
    
    %Definition of the parameters of the algorithm
    L = theta_g - theta_i;  %Displacement
    
    
    % ---------------------------------------------------------------------------------------------------------------
    % ----------------------------- Enter your parameters here --------------------------------------
    % ----------------------------------------------------------------------------------------------------------------

    
    dtheta = 0;     %Velocity at the end of the first blend and the linear part
    ddtheta = 0;  %Acceleration in the blend
    
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
    for t=0+h:h:T
       trajectory(1,j) = t;
       joint_velocity(1,j) = t;
       joint_acceleration(1,j) = t;

       
% ---------------------------------------------------------------------------------------------------------------
% -----------------Insert the correct values of time and the correct trajectory equation for the different regions --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

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
            elseif (t<=T) && (t >=(T-ta))
              trajectory(i+1,j) = 0;
              joint_velocity(i+1,j) = 0;
              joint_acceleration(i+1,j) = 0; 
            end
        end

       j = j+1; 
    end

end