%Simon Michaud
%Creation: 2018-10-09
%Modifications 2018-10-09

function trajectory = Jaco6DOFSTrajectoryPlanner(theta_i, theta_g, desired_time)

% theta_i = [180,180,180,180,180,180];
% theta_g = [182, 189, 184, 110, 182,182];
% desired_time = 1;

JOINTS = 6;

h = desired_time/(desired_time*100);

for i=1:JOINTS
   a0(i) = theta_i(i);
   a1(i) = 0;
   a2(i) = (3/desired_time^2)*(theta_g(i)-theta_i(i));
   a3(i) = -(2/desired_time^3)*(theta_g(i) - theta_i(i));
end

j= 1;
for t=0+h:h:desired_time
   trajectory(1,j) = t;
   joint_velocity(1,j) = t;
   joint_acceleration(1,j) = t;
   
   for i = 1:JOINTS
      trajectory(i+1,j) = a0(i) + a1(i)*t + a2(i)*t^2 + a3(i)*t^3;
      joint_velocity(i+1,j) = a1(i) + 2*a2(i)*t + 3*a3(i)*t^2;
      joint_acceleration(i+1,j) = 2*a2(i) + 6*a3(i)*t;
   end
   j= j+1;
end

end