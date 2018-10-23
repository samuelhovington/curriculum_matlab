%Created on 2018-10-22 by Simon Michaud @Kinova
%Modified on 2018-10-23

function trajectory = Jaco6DOFSTrajectoryPlannerLBP(theta_i, theta_g, T)
% clear all;

% theta_i = [180,180,180,180,180,180];
% theta_g = [180,190,180,180,180,180];
% % theta_i = 180;
% % theta_g = 190;
% T = 2;
    JOINTS = 6;
    ta = T/3;
    h = T/(T*100);

    L = theta_g - theta_i;
    dtheta = 3*L/(2*T);
    ddtheta = 9*L/(2*T^2);
    for i = 1: JOINTS
        a0(i) = theta_i(i);
        a1(i) = 0;
        a2(i) = dtheta(i)/(2*ta);
        b0(i) = theta_i(i) - dtheta(i)*ta/2;
        b1(i) = dtheta(i);
        c0(i) = theta_g(i) - (dtheta(i)*T^2)/(2*ta);
        c1(i) = dtheta(i)*T/ta;
        c2(i) = -dtheta(i)/(2*ta);
    end
    j= 1;
    for t=0+h:h:T
       trajectory(1,j) = t;
       joint_velocity(1,j) = t;
       joint_acceleration(1,j) = t;

        for i = 1:JOINTS
            if (t<ta) && (t>=0)
              trajectory(i+1,j) = a0(i) + a1(i)*t + a2(i)*t^2;
              joint_velocity(i+1,j) = a1(i) + 2*a2(i)*t ;
              joint_acceleration(i+1,j) = 2*a2(i);
            elseif (t<(T-ta)) && (t>=ta)
              trajectory(i+1,j) = b0(i) + b1(i)*t;
              joint_velocity(i+1,j) = b1(i);
              joint_acceleration(i+1,j) =0; 
            elseif (t<=T) && (t >=(T-ta))
              trajectory(i+1,j) = c0(i) + c1(i)*t + c2(i)*t^2;
              joint_velocity(i+1,j) = c1(i) + 2*c2(i)*t ;
              joint_acceleration(i+1,j) = 2*c2(i); 
            end
        end

       j = j+1; 
    end

end