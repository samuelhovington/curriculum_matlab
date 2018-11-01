%Function that takes the number of DOF, the classic DH parameters, the
%transformation matrix between the robot's world frame and the frame 0 and
%the angular position of the robot in angles of the DH algorithm
%Created on 2018-10-22 by Simon Michaud @Kinova
%Modifications
clear all;
% function J = Jacobian(convention, DOF, DH, TW0,theta, angleUnit)
convention = 'Modified';
% convention = 'Classic';
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;

DOF = 6;
    DH = [  0,      0,      -D1;
            pi/2,   0,      0;
            pi ,    D2,     -e2;
            pi/2,   0,      -(D3+D4);
            pi/2,   0,      0;
            pi/2,   0,      (D5+D6)];
%    DH = [  pi/2,   0,      -D1;
%             pi,     D2,      0;
%             pi/2 ,  0,     -e2;
%             pi/2,   0,      -(D3+D4);
%             pi/2,   0,      0;
%             pi,     0,      -(D5+D6)];
TW0 = [1   0   0   0;
        0   -1  0   0;
        0   0   -1  0;
        0   0   0   1];
theta = [180,270,90,270,270,270];
q = [theta(1)+180, theta(2)+90, theta(3)+90, theta(4), theta(5)-180, -(theta(6)+90)];
% q = [theta(1)+180, theta(2)+90, theta(3)+90, theta(4), theta(5), theta(6)-90];

angleUnit = 'Degrees';


%% Parameters for the creation of the jacobian
 for i=1:DOF
   alpha(i) = DH(i,1);
   d(i) = DH(i,3);
   a(i) = DH(i,2);
%    theta(i) = DH(i,4);
end

if strcmp(angleUnit, 'Degrees')
        q = q*pi/180;
        angleUnit = 'Radians';
end
    
%% Creation of the transformation matrices between frames
    if strcmp(convention,'Modified')
        %Transformation matrix ^{n-1}T^{n}
        for i=1:6
            T(:,:,i)=[cos(q(i)) -sin(q(i)) 0 a(i);...
                sin(q(i))*cos(alpha(i)) cos(alpha(i))*cos(q(i)) -sin(alpha(i)) -d(i)*sin(alpha(i)); ...
                sin(q(i))*sin(alpha(i)) cos(q(i))*sin(alpha(i)) cos(alpha(i)) d(i)*cos(alpha(i));...
                0 0 0 1];
        end

    elseif strcmp(convention, 'Classic')
        %Transformation matrix ^{n-1}T^{n}
       for i=1:DOF
            T(:,:,i)=[cos(q(i)) -sin(q(i))*cos(alpha(i)) sin(q(i))*sin(alpha(i)) a(i)*cos(q(i));...
                sin(q(i)) cos(alpha(i))*cos(q(i)) -cos(q(i))*sin(alpha(i)) a(i)*sin(q(i)); ...
                0 sin(alpha(i)) cos(alpha(i)) d(i);...
                0 0 0 1];
        end
    end
    
%% Creation of the transformation matrices from the world frame of the robot
   T0(:,:,1) = TW0 *T(:,:,1);
    for i = 2:DOF
        T0(:,:,i) = T0(:,:,i-1) *T(:,:,i);
    end

%% Creation of the Jacobian
if strcmp(convention, 'Classic')
    %Making the first column of the Jacobian
    Jacobcol1=[cross([TW0(1,3); TW0(2,3); TW0(3,3)], [T0(1,4,DOF); T0(2,4,DOF); T0(3,4,DOF)]); TW0(1,3); TW0(2,3); TW0(3,3)];
    J(:,1) = Jacobcol1;
    for i = 1:DOF
        Jacobcol(:,:,i)=[cross([T0(1,3,i); T0(2,3,i); T0(3,3,i)], [T0(1,4,DOF)-T0(1,4,i); T0(2,4,DOF)-T0(2,4,i); T0(3,4,DOF)-T0(3,4,i)]); T0(1,3,i); T0(2,3,i); T0(3,3,i)];
    end

    for i = 1:(DOF-1)
        J(:,i+1)=Jacobcol(:,1,i);
    end
elseif  strcmp(convention, 'Modified')
    Jacobcol1=[cross([TW0(1,3); TW0(2,3); TW0(3,3)], [T0(1,4,DOF); T0(2,4,DOF); T0(3,4,DOF)]); TW0(1,3); TW0(2,3); TW0(3,3)];
    J(:,1) = Jacobcol1;
    for i = 1:DOF
        Jacobcol(:,:,i)=[cross([T0(1,3,i); T0(2,3,i); T0(3,3,i)], [T0(1,4,DOF)-T0(1,4,i); T0(2,4,DOF)-T0(2,4,i); T0(3,4,DOF)-T0(3,4,i)]); T0(1,3,i); T0(2,3,i); T0(3,3,i)];
    end

    for i = 2:DOF
        J(:,i)=Jacobcol(:,1,i);
    end
    
    
else
    'Choose a convention between Classic and Modified'
end
[0.2736   -0.3111    0.3111   -0.2638   -0.0000    0.0000;
   -0.4100   -0.0000    0.0000   -0.0000    0.0000   -0.0000;
         0   -0.4100    0.0000   -0.0000    0.2638   -0.0000;
         0   -0.0000    0.0000   -0.0000    1.0000    0.0000;
         0    1.0000   -1.0000    0.0000    0.0000   -1.0000;
   -1.0000   -0.0000    0.0000    1.0000    0.0000    0.0000]
