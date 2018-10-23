%Function that takes the number of DOF, the classic DH parameters, the
%transformation matrix between the robot's world frame and the frame 0 and
%the angular position of the robot in angles of the DH algorithm
%Created on 2018-10-22 by Simon Michaud @Kinova
%Modifications

function J = Jacobian(DOF, DH, TW0, theta)
% Parameters for the creation of the jacobian
for i=1:DOF
   alpha(i) = DH(i,3);
   d(i) = DH(i,2);
   a(i) = DH(i,1);
end

% ----------------------------------------------------------------------------------------------------------------
% ----------------------- Create the transformation matrices between frames --------------------------------------
% ----------------------------------------------------------------------------------------------------------------
    for i=1:DOF
        T(:,:,i)=[  0 0 0 0 ...
                    0 0 0 0 ...
                    0 0 0 0 ...
                    0 0 0 0];
    end

% ----------------------------------------------------------------------------------------------------------------
% ---- Create the transformation matrices from the world frame of the robot --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

T0(:,:,1) = TW0 *T(:,:,1);
    for i = 2:DOF
        T0(:,:,i) = T0(:,:,i-1) *T(:,:,i);
    end

% ----------------------------------------------------------------------------------------------------------------
% ----------------------------------------------------- Create the Jacobian --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

Jacobcol1=[cross([TW0(1,3); TW0(2,3); TW0(3,3)], [T0(1,4,DOF); T0(2,4,DOF); T0(3,4,DOF)]); TW0(1,3); TW0(2,3); TW0(3,3)];
J(:,1) = Jacobcol1;
for i = 1:DOF
    Jacobcol(:,:,i)=[cross([T0(1,3,i); T0(2,3,i); T0(3,3,i)], [T0(1,4,DOF)-T0(1,4,i); T0(2,4,DOF)-T0(2,4,i); T0(3,4,DOF)-T0(3,4,i)]); T0(1,3,i); T0(2,3,i); T0(3,3,i)];
end

for i = 1:(DOF-1)
    J(:,i+1)=Jacobcol(:,1,i)
end
end