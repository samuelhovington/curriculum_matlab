%Creation: 2018-10-18 by Simon Michaud @Kinova
%Modifications 2018-11-07

%Trajectory planner using a polynomial function in the joint space scheme

%Arguments: Initial angular position of the robot, goal angular position,
            %desired time
%Returns:   7x100*t(in sec) matrix that define the trajectory, with the 
            %first row being the time and the 6 other being the angular 
            %positions of the robot
function trajectory = TrajectoryPlanner_6DOFS_CP_complete(theta_i, theta_g, desired_time)
JOINTS = 6;

h = desired_time/(desired_time*100);

%Definition of the parameters going into the equations
for i=1:JOINTS
   a0(i) = theta_i(i);
   a1(i) = 0;
   a2(i) = (3/desired_time^2)*(theta_g(i)-theta_i(i));
   a3(i) = -(2/desired_time^3)*(theta_g(i) - theta_i(i));
end

%Generating the trajectory
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