%Created 2018-10-22 by Simon Michaud @Kinova
%Modified 2018-11-07 @10:09 am

%Jacobian function that generates the Jacobian matrix of the given angular
%position.

%Arguments: Convention of the DH parameters, number of degrees of freedom,
            %the DH parameters, the transformation matrix from world frame
            %to the 0 DH frame, the angular position, the units of the
            %angles of the angular position
%Returns:   DOFxDOF matrix that define the Jacobian of the given angular
            %position

function J = Jacobian_to_complete(convention, DOF, DH, TW0, q, angleUnit)
%% Parameters for the creation of the jacobian
    for i=1:DOF
       alpha(i) = DH(i,1);
       d(i) = DH(i,3);
       a(i) = DH(i,2);
    end
    if strcmp(angleUnit, 'Degrees')
        q = q*pi/180;
        angleUnit = 'Radians'
    end
    
% This section should be the same as in the foward_kinematics_to_complete
  if strcmp(convention,'Modified')
% ----------------------------------------------------------------------------------------------------------------
% ---------------------------- Add the Transformation matrix ^{n-1}T^{n} --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

        % Use only radians
        for i=1:6
            T(:,:,i)=[  0 0 0 0; ...
                        0 0 0 0; ...
                        0 0 0 0; ...
                        0 0 0 0];
        end

    elseif strcmp(convention, 'Classic')

% ----------------------------------------------------------------------------------------------------------------
% ------------------------------- Add the Transformation matrix ^{n-1}T^{n} --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

        % Use only radians
        for i=1:6
            T(:,:,i)=[  0 0 0 0; ...
                        0 0 0 0; ...
                        0 0 0 0; ...
                        0 0 0 0];
        end
    end
% ----------------------------------------------------------------------------------------------------------------
% --------------- Create the transformation matrices from the world frame of the robot ---------------------------
% ----------------------------------------------------------------------------------------------------------------

    T0(:,:,1) = TW0 *T(:,:,1);
    for i = 2:DOF
        T0(:,:,i) = T0(:,:,i-1) *T(:,:,i);
    end

% ----------------------------------------------------------------------------------------------------------------
% --------------------------- Change the following lines to create the Jacobian ----------------------------------
% ----------------------------------------------------------------------------------------------------------------
    if strcmp(convention, 'Classic')
% Enter the first column of the jacobian in Jacobol1
        Jacobcol1=[0;0;0;0;0;0];
        J(:,1) = Jacobcol1;
        for i = 1:DOF
% Enter the algorithm for the columns of the Jacobian here
            Jacobcol(:,:,i)=[0;0;0;0;0;0];
        end
% Adjust the loop correctly to place the column in the right order
        for i = 1:DOF
            J(:,i)=Jacobcol(:,1,i)
        end
        
% Add correct signs to represent the good orientation of rotation
        J(:,1) = J(:,1);
        J(:,2) = J(:,2);
        J(:,3) = J(:,3);
        J(:,4) = J(:,4);
        J(:,5) = J(:,5);
        J(:,6) = J(:,6);

    elseif  strcmp(convention, 'Modified')
% Enter the first column of the jacobian in Jacobol1
        Jacobcol1=[0;0;0;0;0;0];
        J(:,1) = Jacobcol1;
        for i = 1:DOF
% Enter the algorithm for the columns of the Jacobian here
            Jacobcol(:,:,i)=[0;0;0;0;0;0];
        end
% Adjust the loop correctly to place the column in the right order       
        for i = 2:DOF
            J(:,i)=Jacobcol(:,1,i);
        end
        
% Add correct signs to represent the good orientation of rotation
        J(:,1) = J(:,1);
        J(:,2) = J(:,2);
        J(:,3) = J(:,3);
        J(:,4) = J(:,4);
        J(:,5) = J(:,5);
        J(:,6) = J(:,6);
    else
        'Choose a convention between Classic and Modified'
    end
end