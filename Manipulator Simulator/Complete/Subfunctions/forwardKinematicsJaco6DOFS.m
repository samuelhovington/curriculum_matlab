%Jaco 6 DOF Spherical wrist, function that returns the position of the DH
%frames of the robot from the angular positions of the joints
%Created 2018-09-25 by Simon Michaud @Kinova
%Modified 2018-10-05 @10:11 am

function [coordinates] = forwardKinematicsJaco6DOFS(Q1,Q2,Q3,Q4,Q5,Q6)
%Difference between physical angles Q and Dh algorithm angles q
q(1) = Q1;
q(2) = Q2-90;
q(3) = Q3+90;
q(4) = -Q4;
q(5) = -Q5;
q(6) = Q6;

%Robot geometric parameters
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;

%DH parameters
DH = [0, 0, -D1, q(1); -pi/2, 0, 0, q(2); pi ,D2, -e2, q(3); 3*pi/2, 0, (D3+D4), q(4); pi/2, 0, 0, q(5); 3*pi/2, 0, -(D5+D6), q(6)];

alpha(1)=DH(1,1);
alpha(2)=DH(2,1);
alpha(3)=DH(3,1);
alpha(4)=DH(4,1);
alpha(5)=DH(5,1);
alpha(6)=DH(6,1);

d(1)=DH(1,3);
d(2)=DH(2,3);
d(3)=DH(3,3);
d(4)=DH(4,3);
d(5)=DH(5,3);
d(6)=DH(6,3);

a(1)=DH(1,2);
a(2)=DH(2,2);
a(3)=DH(3,2);
a(4)=DH(4,2);
a(5)=DH(5,2);
a(6)=DH(6,2);


%Transformation matrix ^{n-1}T^{n}
for i=1:6
    T(:,:,i)=[cosd(q(i)) -sind(q(i)) 0 a(i);...
        sind(q(i))*cos(alpha(i)) cos(alpha(i))*cosd(q(i)) -sin(alpha(i)) -d(i)*sin(alpha(i)); ...
        sind(q(i))*sin(alpha(i)) cosd(q(i))*sin(alpha(i)) cos(alpha(i)) d(i)*cos(alpha(i));...
        0 0 0 1];
end

%Transformation matrices from the base to the joint i
T0=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];
T1=T0*T(:,:,1);
T2=T1*T(:,:,2);
T3=T2*T(:,:,3);
T4=T3*T(:,:,4);
T5=T4*T(:,:,5);
T6=T5*T(:,:,6);

%Jacobian
Jacobcol1=[cross([0;0;-1], [T6(1,4); T6(2,4); T6(3,4)]); 0; 0; -1];
Jacobcol2=[cross([T1(1,3); T1(2,3); T1(3,3)], [T6(1,4)-T1(1,4); T6(2,4)-T1(2,4); T6(3,4)-T1(3,4)]); T1(1,3); T1(2,3); T1(3,3)];
Jacobcol3=[cross([T2(1,3); T2(2,3); T2(3,3)], [T6(1,4)-T2(1,4); T6(2,4)-T2(2,4); T6(3,4)-T2(3,4)]); T2(1,3); T2(2,3); T2(3,3)];
Jacobcol4=[cross([T3(1,3); T3(2,3); T3(3,3)], [T6(1,4)-T3(1,4); T6(2,4)-T3(2,4); T6(3,4)-T3(3,4)]); T3(1,3); T3(2,3); T3(3,3)];
Jacobcol5=[cross([T4(1,3); T4(2,3); T4(3,3)], [T6(1,4)-T4(1,4); T6(2,4)-T4(2,4); T6(3,4)-T4(3,4)]); T4(1,3); T4(2,3); T4(3,3)];
Jacobcol6=[cross([T5(1,3); T5(2,3); T5(3,3)], [T6(1,4)-T5(1,4); T6(2,4)-T5(2,4); T6(3,4)-T5(3,4)]); T5(1,3); T5(2,3); T5(3,3)];
Jacob=[Jacobcol1 Jacobcol2 Jacobcol3 Jacobcol4 Jacobcol5 Jacobcol6];

%Physical positions of the origins of the frames to view the robot
J6 = [T6(1,4);T6(2,4);T6(3,4)];
J5 = [T5(1,4);T5(2,4);T5(3,4)];
J3 = [T3(1,4);T3(2,4);T3(3,4)];
J1 = [T1(1,4);T1(2,4);T1(3,4)];
J0 = [T0(1,4);T0(2,4);T0(3,4)];
coordinates = [J0,J1,J3,J5,J6];
end
