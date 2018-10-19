%Simon Michaud
%Creation: 2018-10-18
%Modifications 2018-10-18
%Trajectory planner with 1 via point in the middle
yo
function trajectory = Jaco6DOFSTrajectoryPlannerCPV(theta_i, theta_v, theta_g, desired_time)

% clear all
% theta_i = [180,180,180,180,180,180];
% theta_g = [180,0,180,180,180,180];
% theta_v = [180,90,180,180,180,180];
% desired_time = 1;

JOINTS = 6;

h = desired_time/(desired_time*100);

for i=1:JOINTS
   a10(i) = theta_i(i);
   a11(i) = 0;
   a12(i) = (12*theta_v(i)-3*theta_g(i)-9*theta_i(i))/(4*desired_time^2);
   a13(i) = (-8*theta_v(i)+3*theta_g(i)+5*theta_i(i))/(4*desired_time^3);
   a20(i) = theta_v(i);
   a21(i) = (3*theta_g(i)-3*theta_i(i))/(4*desired_time);
   a22(i) = (-12*theta_v(i)+6*theta_g(i)+6*theta_i(i))/(4*desired_time^2);
   a23(i) = (8*theta_v(i)-5*theta_g(i)-3*theta_i(i))/(4*desired_time^3);
end

j= 1;
for t=0+h:h:desired_time
   trajectory(1,j) = t;
   joint_velocity(1,j) = t;
   joint_acceleration(1,j) = t;
   
   for i = 1:JOINTS
      trajectory(i+1,j) = a10(i) + a11(i)*t + a12(i)*t^2 + a13(i)*t^3;
      joint_velocity(i+1,j) = a11(i) + 2*a12(i)*t + 3*a13(i)*t^2;
      joint_acceleration(i+1,j) = 2*a12(i) + 6*a13(i)*t;
   end
   j= j+1;
end
for t=0+h:h:desired_time
   trajectory(1,j) = t + desired_time;
   joint_velocity(1,j) = t + desired_time;
   joint_acceleration(1,j) = t + desired_time;
   
   for i = 1:JOINTS
      trajectory(i+1,j) = a20(i) + a21(i)*t + a22(i)*t^2 + a23(i)*t^3;
      joint_velocity(i+1,j) = a21(i) + 2*a22(i)*t + 3*a23(i)*t^2;
      joint_acceleration(i+1,j) = 2*a22(i) + 6*a23(i)*t;
   end
   j= j+1;
end

end