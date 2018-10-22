%Jaco 6 DOF Spherical wrist, function that returns the position of the DH
%frames of the robot from the angular positions of the joints
%Created 2018-09-25 by Simon Michaud @Kinova
%Modified 2018-10-05 @10:11 am

function [coordinates] = forwardKinematicsJaco6DOFS(q,DH,T0,convension)
   for i=1:6
       alpha(i) = DH(i,1);
       d(i) = DH(i,3);
       a(i) = DH(i,2);
   end    

    if strcmp(convension,'Modified')
        %Transformation matrix ^{n-1}T^{n}
        for i=1:6
            T(:,:,i)=[cosd(q(i)) -sind(q(i)) 0 a(i);...
                sind(q(i))*cos(alpha(i)) cos(alpha(i))*cosd(q(i)) -sin(alpha(i)) -d(i)*sin(alpha(i)); ...
                sind(q(i))*sin(alpha(i)) cosd(q(i))*sin(alpha(i)) cos(alpha(i)) d(i)*cos(alpha(i));...
                0 0 0 1];
        end

    elseif strcmp(convension, 'Classic')
        
        %Transformation matrix ^{n-1}T^{n}
        for i=1:6
            T(:,:,i)=[cosd(q(i)) -sind(q(i))*cos(alpha(i)) sind(q(i))*sin(alpha(i)) a(i)*cosd(q(i));...
                sind(q(i)) cos(alpha(i))*cosd(q(i)) -cosd(q(i))*sin(alpha(i)) a(i)*sind(q(i)); ...
                0 sin(alpha(i)) cos(alpha(i)) d(i);...
                0 0 0 1];
        end
    end
    %Transformation matrices from the base to the joint i
    % T0=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];
    T0(:,:,1) = T0 *T(:,:,1);
    for i = 2:6
        T0(:,:,i) = T0(:,:,i-1) *T(:,:,i);
    end
        
        %Physical positions of the origins of the frames to view the robot
        J6 = [T0(1,4,6);T0(2,4,6);T0(3,4,6)];
        J5 = [T0(1,4,5);T0(2,4,5);T0(3,4,5)];
        J4 = [T0(1,4,4);T0(2,4,4);T0(3,4,4)];
        J3 = [T0(1,4,3);T0(2,4,3);T0(3,4,3)];
        J2 = [T0(1,4,2);T0(2,4,2);T0(3,4,2)];
        J1 = [T0(1,4,1);T0(2,4,1);T0(3,4,1)];
        J0 = [T0(1,4);T0(2,4);T0(3,4)];
        coordinates = [J0,J1,J2,J3,J4,J5,J6];
end
