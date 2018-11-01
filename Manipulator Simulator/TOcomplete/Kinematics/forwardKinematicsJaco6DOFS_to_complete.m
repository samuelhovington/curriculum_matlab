%Jaco 6 DOF Spherical wrist, function that returns the position of the DH
%frames of the robot from the angular positions of the joints. The
%arguments of the function are the convention of the DH parameters, the DH
%parameters, the transformation matrix from the world frame to the 0 frame,
%the angular position of the joint and their unit (degrees or radians)
%Created 2018-09-25 by Simon Michaud @Kinova
%Modified 2018-10-31 @10:11 am

function [coordinates] = forwardKinematicsJaco6DOFS_to_complete(convention, DH, T0, q, angleUnit)
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
            T(:,:,i)=[  0 0 0 0 ...
                        0 0 0 0 ...
                        0 0 0 0 ...
                        0 0 0 0];
        end

    elseif strcmp(convention, 'Classic')

% ----------------------------------------------------------------------------------------------------------------
% ---------------------------- Create the Transformation matrix ^{n-1}T^{n} --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

        % Make sure to use cosd and sind when angles are in degree
        for i=1:6
            T(:,:,i)=[  0 0 0 0 ...
                        0 0 0 0 ...
                        0 0 0 0 ...
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
    J6 = [T0(1:3,4,6)];
        J5 = [T0(1:3,4,5)];
        J4 = [T0(1:3,4,4)];
        J3 = [T0(1:3,4,3)];
        J2 = [T0(1:3,4,2)];
        J1 = [T0(1:3,4,1)];
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
    X = [r6_0;EulerXYZ]
        
end
