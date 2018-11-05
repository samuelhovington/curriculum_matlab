%Function that takes the convention of the DH parameters, the number of DOF,
%the classic DH parameters, the transformation matrix between the robot's 
%world frame and the frame 0, the angular position of the robot in angles 
%of the DH algorithm and the unit of this angular position (degrees or
%radians)
%Created 2018-10-22 by Simon Michaud @Kinova
%Modified 2018-10-31 @10:09 am

function J = Jacobian_complete(convention, DOF, DH, TW0, q, angleUnit)
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
        % Algorithm for the columns of the Jacobian
            Jacobcol(:,:,i)=[cross([T0(1,3,i); T0(2,3,i); T0(3,3,i)], [T0(1,4,DOF)-T0(1,4,i); T0(2,4,DOF)-T0(2,4,i); T0(3,4,DOF)-T0(3,4,i)]); T0(1,3,i); T0(2,3,i); T0(3,3,i)];
        end
        
        % Making the loop to place the column in the right order
        for i = 1:(DOF-1)
            J(:,i+1)=Jacobcol(:,1,i);
        end
        % Correcting the orientation of rotation.  Varies due to DH frames
        J(:,1) = J(:,1);
        J(:,2) = J(:,2);
        J(:,3) = J(:,3);
        J(:,4) = J(:,4);
        J(:,5) = J(:,5);
        J(:,6) = J(:,6);
        
    elseif  strcmp(convention, 'Modified')
        %Making the first column of the Jacobian
        Jacobcol1=[cross([TW0(1,3); TW0(2,3); TW0(3,3)], [T0(1,4,DOF); T0(2,4,DOF); T0(3,4,DOF)]); TW0(1,3); TW0(2,3); TW0(3,3)];
        J(:,1) = Jacobcol1;
        for i = 1:DOF
        % Algorithm for the columns of the Jacobian
            Jacobcol(:,:,i)=[cross([T0(1,3,i); T0(2,3,i); T0(3,3,i)], [T0(1,4,DOF)-T0(1,4,i); T0(2,4,DOF)-T0(2,4,i); T0(3,4,DOF)-T0(3,4,i)]); T0(1,3,i); T0(2,3,i); T0(3,3,i)];
        end
        
        % Making the loop to place the column in the right order
        for i = 2:DOF
            J(:,i)=Jacobcol(:,1,i);
        end 
        
        % Correcting the orientation of rotation. Varies due to DH frames
        J(:,1) = J(:,1);
        J(:,2) = J(:,2);
        J(:,3) = J(:,3);
        J(:,4) = J(:,4);
        J(:,5) = J(:,5);
        J(:,6) = -J(:,6);
    else
        'Choose a convention between Classic and Modified'
    end
end