%Jaco 6 DOF Spherical wrist, function that returns the position of the DH
%frames of the robot from the angular positions of the joints
%Created 2018-09-25 by Simon Michaud @Kinova
%Modified 2018-10-05 @10:11 am

function [coordinates] = forwardKinematicsJaco6DOFS(q,DH,T0,convension)
    
    % store the DH parameters into variable to make it clearer
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
        
    if strcmp(convension,'Modified')
    % ----------------------------------------------------------------------------------------------------------------
    % ---------------------------- Create the Transformation matrix ^{n-1}T^{n} --------------------------------------
    % ----------------------------------------------------------------------------------------------------------------

            % Make sure to use cosd and sind where angles are in degree 
            for i=1:6
                T(:,:,i)=[  0 0 0 0 ...
                            0 0 0 0 ...
                            0 0 0 0 ...
                            0 0 0 0];
            end

    end

    elseif strcmp(convension, 'Classic')

    % ----------------------------------------------------------------------------------------------------------------
    % ---------------------------- Create the Transformation matrix ^{n-1}T^{n} --------------------------------------
    % ----------------------------------------------------------------------------------------------------------------

        % Make sure you use cosd and sind because the function take degree
        % in input
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
        
end
