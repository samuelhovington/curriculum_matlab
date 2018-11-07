%Created on 2018-10-22 by Simon Michaud @Kinova
%Modified on 2018-11-07

%Trajectory planner using a linear function with blends in the joint space 
%scheme. 

%Arguments: Initial angular position of the robot, goal angular position, 
            %desired time
%Returns:   7x100*t(in sec)*2 matrix that define the trajectory, with the 
            %first row being the time and the 6 other being the angular 
            %positions of the robot

function trajectory =TrajectoryPlanner_6DOFS_LB_J_complete(theta_i, theta_g, T)
    JOINTS = 6;
    % Definition of the time parameters
    ta = T/3;               %ta is the time in the acceleration blend
    h = T/(T*100);          %h is the time step for the trajectory matrix
    
    %Definition of the parameters of the algorithm
    L = theta_g - theta_i;  %Displacement
    dtheta = 3*L/(2*T);     %Velocity at the end of the first blend and the linear part
    ddtheta = 9*L/(2*T^2);  %Acceleration in the blend
    
    % Parameters entered in the trajectory equation
    for i = 1: JOINTS
        a0(i) = theta_i(i);
        a1(i) = 0;
        a2(i) = ddtheta(i)/2;
        b0(i) = theta_i(i) - dtheta(i)*ta/2;
        b1(i) = dtheta(i);
        c0(i) = theta_g(i) - (dtheta(i)*T^2)/(2*ta);
        c1(i) = dtheta(i)*T/ta;
        c2(i) = -ddtheta(i)/2;
    end
    
    % Generation of the trajectory
    j= 1;
    for t=0+h:h:T
       trajectory(1,j) = t;
       joint_velocity(1,j) = t;
       joint_acceleration(1,j) = t;

        for i = 1:JOINTS
            %Verification that the trajectory is in the acceleration phase
            if (t<ta) && (t>=0)
              trajectory(i+1,j) = a0(i) + a1(i)*t + a2(i)*t^2;
              joint_velocity(i+1,j) = a1(i) + 2*a2(i)*t ;
              joint_acceleration(i+1,j) = 2*a2(i);
            
            %Verification that the trajectory is in the linear phase
            elseif (t<(T-ta)) && (t>=ta)
              trajectory(i+1,j) = b0(i) + b1(i)*t;
              joint_velocity(i+1,j) = b1(i);
              joint_acceleration(i+1,j) =0;
            
            %Verification that the trajectory is in the deceleration phase
            elseif (t<=T) && (t >=(T-ta))
              trajectory(i+1,j) = c0(i) + c1(i)*t + c2(i)*t^2;
              joint_velocity(i+1,j) = c1(i) + 2*c2(i)*t ;
              joint_acceleration(i+1,j) = 2*c2(i); 
            end
        end

       j = j+1; 
    end

end