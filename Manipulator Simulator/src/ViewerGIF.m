clear all; close all; clc;
Rad2Deg = 180/pi;

theta_i = [180,180,180,180,180,180];
theta_g = [180,0,180,180,0,180];
theta_v = [180,0,180,180,0,180];
desired_time = 1;

JntPos = Jaco6DOFSTrajectoryPlannerCP(theta_i, theta_g, desired_time);
% Pee = load ('Pee.mat');

% Q1 = Rad2Deg.*JntPos(2,:);
% Q2 = Rad2Deg.*JntPos(3,:);
% Q3 = Rad2Deg.*JntPos(4,:);
% Q4 = Rad2Deg.*JntPos(5,:);
% Q5 = Rad2Deg.*JntPos(6,:);
% Q6 = Rad2Deg.*JntPos(7,:);

Q1 = JntPos(2,:);
Q2 = JntPos(3,:);
Q3 = JntPos(4,:);
Q4 = JntPos(5,:);
Q5 = JntPos(6,:);
Q6 = JntPos(7,:);



for index = 1:length(JntPos(1,:))
   coordinates = forwardKinematicsJaco6DOFS(Q1(index), Q2(index), Q3(index), Q4(index), Q5(index), Q6(index)); 
   EndEffector(index,:)=coordinates(:,5);
   
   h = figure(1);
   
    axis tight manual % this ensures that getframe() returns a consistent size
    filename = 'testAnimated.gif';
    
    
   plot3(coordinates(1,:),coordinates(2,:),coordinates(3,:))
   hold on
   view(210,40) 
   set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
%    set ( gca, 'xdir', 'reverse' )
%     set ( gca, 'ydir', 'reverse' )
   plot3(coordinates(1,1),coordinates(2,1),coordinates(3,1), '-r*')
   axis equal
   set(findall(gca, 'Type', 'Line'),'LineWidth',2);
   xlabel('X')
   ylabel('Y')
   xlim([-1 1])
   ylim([-1 1])
   zlim([-1.3 1.3])
   plot3(EndEffector(:,1),EndEffector(:,2),EndEffector(:,3))
   %plot3(Pee.ans(2,:),Pee.ans(3,:),Pee.ans(4,:))
   grid on
   hold off
   
   % Capture the plot as an image 
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if index == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
      end 
     
    
end