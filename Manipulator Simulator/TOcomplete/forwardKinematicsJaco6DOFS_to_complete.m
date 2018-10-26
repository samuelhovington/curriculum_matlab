%Jaco 6 DOF Spherical wrist, function that returns the position of the DH
%frames of the robot from the angular positions of the joints
%Created 2018-09-25 by Simon Michaud @Kinova
%Modified 2018-10-05 @10:11 am

function [coordinates] = forwardKinematicsJaco6DOFS(q,DH,T0,convension)
    
    % store the DH parameters into variable to make it clearer
    for i=1:6
       alpha(i) = DH(i,1);
       d(i) = DH(i,3);
       a(i) = DH(i,2);
   end
        
    if strcmp(convension,'Modified')
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

    elseif strcmp(convension, 'Classic')

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

    %Physical positions of the origins of the frames to view the robot
    J6 = [T6(1,4);T6(2,4);T6(3,4)];
    J5 = [T5(1,4);T5(2,4);T5(3,4)];
    J4 = [T4(1,4);T4(2,4);T4(3,4)];
    J3 = [T3(1,4);T3(2,4);T3(3,4)];
    J2 = [T2(1,4);T2(2,4);T2(3,4)];
    J1 = [T1(1,4);T1(2,4);T1(3,4)];
    J0 = [T0(1,4);T0(2,4);T0(3,4)];
    coordinates = [J0,J1,J2,J3,J4,J5,J6];
    
% ----------------------------------------------------------------------------------------------------------------
% ------------- Create the vector of the end effector position in term of the base --------------------------------------
% ----------------------------------------------------------------------------------------------------------------

    %Definition of the rotation matrices and the position vectors from the
    %transformation matrix to obtain the position and orientation of the
    %effector
    R6_0 = [T6(1,1), T6(1,2), T6(1,3);T6(2,1),T6(2,2),T6(2,3);T6(3,1),T6(3,2),T6(3,3)];
    r6_0 = [T6(1,4), T6(2,4), T6(3,4)];
    EulerXYZ = MatRotationToEuler(R6_0);
    X = [r6_0;EulerXYZ]
        
end
