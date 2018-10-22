%Function that takes the number of DOF, the classic DH parameters, the
%transformation matrix between the robot's world frame and the frame 0 and
%the angular position of the robot in angles of the DH algorithm
%Created on 2018-10-22 by Simon Michaud @Kinova
%Modifications

theta = [180,270,90,270,270,270];
%dH=[0 -0.275500000000000 1.57079632679490;0 0 1.57079632679490;0 -0.410000000000000 1.57079632679490;0 -0.00980000000000000 1.57079632679490;0 -0.31150000000000000 1.57079632679490;0 0 1.57079632679490;0 -0.263750000000000 pi]
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;

e2 = 0.0098;
Deg2Rad = pi/180;

theta(1) = Deg2Rad * (theta(1) + 180);
theta(2) = Deg2Rad * (theta(2) - 90);
theta(3) = Deg2Rad * (theta(3) - 90);
theta(4) = Deg2Rad * theta(4);
theta(5) = Deg2Rad * theta(5);
theta(6) = Deg2Rad * (theta(6) + 90);
dH = [0 -D1 pi/2; -D2 0 pi; 0 -e2 pi/2; 0 -(D3 + D4) pi/2; 0 0 pi/2; 0 -(D5+D6) pi];

TW0=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];

%function J = Jacobian(DOF, DH, TW0, theta)
%% Parameters for the creation of the jacobian
for i=1:DOF
   alpha(i) = DH(i,1);
   d(i) = DH(i,3);
   a(i) = DH(i,2);
end


%% Creation of the transformation matrices between frames
    for i=1:DOF
        T(:,:,i)=[cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));...
            sin(theta(i)) cos(alpha(i))*cos(theta(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i)); ...
            0 sin(alpha(i)) cos(alpha(i)) d(i);...
            0 0 0 1];
    end

%% Creation of the transformation matrices from the world frame of the robot
T0(:,:,1) = TW0 *T(:,:,1);
    for i = 2:DOF
        T0(:,:,i) = T0(:,:,i-1) *T(:,:,i);
    end

%% Creation of the Jacobian
Jacobcol1=[cross([TW0(1,3); TW0(2,3); TW0(3,3)], [T0(1,4,DOF)-TW0(1,4); T0(2,4,DOF)-TW0(2,4); T0(3,4,DOF)-TW0(3,4)]); TW0(1,3); TW0(2,3); TW0(3,3)];
J(:,1) = Jacobcol1
for i = 1:DOF
    Jacobcol(:,:,i)=[cross([T0(1,3,i); T0(2,3,i); T0(3,3,i)], [T0(1,4,DOF)-T0(1,4,i); T0(2,4,DOF)-T0(2,4,i); T0(3,4,DOF)-T0(3,4,i)]); T0(1,3,i); T0(2,3,i); T0(3,3,i)];
end
for i = 2:DOF
    J(:,i)=Jacobcol(:,:,i-1)
%end