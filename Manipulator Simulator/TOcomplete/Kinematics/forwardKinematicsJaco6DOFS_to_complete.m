%Created 2018-09-25 by Simon Michaud @Kinova
%Modified 2018-10-31 @10:11 am

%Foward kinematics function that returns the cartesian position of the end
%effector from the angular position of the robot

%Arguments: Convention of the DH parameters, the DH parameters, the 
            %transformation matrix from world frame to the 0 DH frame, 
            %the goal cartesian position of the end effector, the guessed
            %angular position and the units of the angles of the angular 
            %position
%Returns:   3x7 vector containing the cartesian position of the angles of
            %the robot where each column is a joint and column 1 is the
            %base


function [coordinates] = forwardKinematicsJaco6DOFS_to_complete(convention, DH, TW0, q, angleUnit)
%% Parameters for the creation of the FK
    % store the DH parameters into variable to make it clearer
    for i=1:6
       alpha(i) = DH(i,1);
       d(i) = DH(i,3);
       a(i) = DH(i,2);
    end
    
    %Verify that the angles entering the computing of the transformation
    %are in radians
    if strcmp(angleUnit, 'degrees')
        q = q*pi/180;
    end
%% Computing of the transformation matrices
    if strcmp(convention,'Modified')
% ----------------------------------------------------------------------------------------------------------------
% ---------------------------- Create the Transformation matrix ^{n-1}T^{n} --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

        % Make sure to use cosd and sind when angles are in degree 
        for i=1:6
           T(:,:,i)=[  0 0 0 0;...
                        0 0 0 0;...
                        0 0 0 0;...
                        0 0 0 0];
        end

    elseif strcmp(convention, 'Classic')

% ----------------------------------------------------------------------------------------------------------------
% ---------------------------- Create the Transformation matrix ^{n-1}T^{n} --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

        % Make sure to use cosd and sind when angles are in degree
        for i=1:6
            T(:,:,i)=[  0 0 0 0;...
                        0 0 0 0;...
                        0 0 0 0;...
                        0 0 0 0];
        end
    end
% ----------------------------------------------------------------------------------------------------------------
% ------------- Create transformation matrices from the base to the joint i --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

    % Make the right operation with the matrix T to create T1 to T6
    T1= T(:,:,1);
    T2= T(:,:,2);
    T3= T(:,:,3);
    T4= T(:,:,4);
    T5= T(:,:,5);
    T6= T(:,:,6);
    
%% Positions and orientation of the joints from transformation matrices
    %Physical positions of the origins of the frames to view the robot
        J6 = [T6(1:3,4)];
        J5 = [T5(1:3,4)];
        J4 = [T4(1:3,4)];
        J3 = [T3(1:3,4)];
        J2 = [T2(1:3,4)];
        J1 = [T1(1:3,4)];
        J0 = [TW0(1:3,4)];
        coordinates = [J0,J1,J2,J3,J4,J5,J6];
    
% ----------------------------------------------------------------------------------------------------------------
% ------------- Create the vector of the end effector position in term of the base --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

    %Definition of the rotation matrices and the position vectors from the
    %transformation matrix to obtain the position and orientation of the
    %effector
    R6_0 = [T6(1:3,1), T6(1:3,2), T6(1:3,3)];
    r6_0 = [T6(1,4), T6(2,4), T6(3,4)];
    EulerXYZ = MatRotationToEuler(R6_0);
    X = [transpose(r6_0);EulerXYZ];
        
end
