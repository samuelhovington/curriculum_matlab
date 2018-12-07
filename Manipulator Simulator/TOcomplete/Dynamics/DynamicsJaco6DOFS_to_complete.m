%Created on 2018-11-28 by Simon Michaud @Kinova
%Modified 

%File that executes the dynamics of a 6 DOF S Ultra Lightweight Robotic Arm
%using the Newton-Euler iterative algorithm.

%Arguments: angle, angular velocity, angular acceleration for every joints
            %and the unit of the angles. The angular velocity and
            %accelerations must be in radians
%Returns:   Torques at every joints

function tau = DynamicsJaco6DOFS_to_complete(theta, dtheta, ddtheta, AngleUnit)
%% Parameters for the Dynamics Algorithm
% ----------------------------------------------------------------------------------------------------------------
% --------------------------------------------- DO NOT CHANGE ----------------------------------------------------
% ----------------------------------------------------------------------------------------------------------------
% Changing angles into Radians
    if strcmp(AngleUnit, 'Degrees')
        theta = theta*pi/180;
        AngleUnit = 'Radians';
    end

%Definition of the masses of every joints
    m = [0.18198+0.570,0.42399+0.570,0.21100+0.570,0.09184+0.357, 0.09184+0.357,0.727+0.357];
    
%Definition of the lengths of the links
    D(1) = 0.2755;
    D(2) = 0.4100;
    D(3) = 0.2073;
    D(4) = 0.1038;
    D(5) = 0.1038;
    D(6) = 0.16;
    e2 = 0.0098;
%Definition of the gravitational acceleration
    g = 9.81;

% Definition of the Inertia matrices. These matrices only work for the used
% DH parameters and used DH frames. If you change the default DH
% parameters, you will have to change those matrices.
    Ipm = [0.0001020703 0.0003248936 0.0003536944 -5.214394e-9 -0.000021186541 -4.141348e-9];
    Ic = [Ipm(2), Ipm(3), Ipm(1), -Ipm(6), Ipm(4), -Ipm(5)];
    I(:,:,1) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
    Ipm = [0.0037862305 0.0037251264 0.000069776668 0 0 5.903745e-8];
    Ic = [Ipm(3), Ipm(2), Ipm(1), Ipm(6), Ipm(5), Ipm(4)];
    I(:,:,2) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
    Ipm = [0.0005109958 0.0004804467 0.00004810136 1.922692e-9 -0.000001586224 1.960234e-9];
    Ic = [Ipm(2), Ipm(3), Ipm(1), -Ipm(6), -Ipm(4), Ipm(5)];
    I(:,:,3) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
    Ipm = [0.0001933181 0.00007567544 0.0001953757 -0.00003628615 -1.204e-8 1.37e-8];
    Ic = [Ipm(3), Ipm(1), Ipm(2), -Ipm(5), Ipm(6), -Ipm(4)];
    I(:,:,4) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
    Ic = [Ipm(3), Ipm(2), Ipm(1), -Ipm(6), -Ipm(5), Ipm(4)];
    I(:,:,5) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
    Ipm = [0.00004659624 0.00002647306 0.00005247987 0.00000413032 -1.0573e-7 -3.244e-8];
    Ic = [Ipm(3), Ipm(1), Ipm(2), Ipm(5), -Ipm(6), -Ipm(4)];
    I(:,:,6) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];

% Vector of the centers of mass of the links from the center of the joints
    rcmi_i(:,:,1) = [0;0;D(1)/2] + [0.23712255; -10.29160997; 69.17216347].*10^-3;
    rcmi_i(:,:,2) = [205.23664355, -0.02629337, 22.30776575].*10^-3;
    rcmi_i(:,:,3) = [0.07073408, 64.37250505, -19.56874086].*10^-3;
    rcmi_i(:,:,4) = [0.01, -13.28, -62.52].*10^-3;
    rcmi_i(:,:,5) = [0.01, D(5) - 62.52, -13.28].*10^-3;
    rcmi_i(:,:,6) = [-6.84, -0.03, 82.22].*10^-3;

% Vector of the joint i from joint i-1
    r_i_im1(:,:,1) = [0;0;0];
    r_i_im1(:,:,2) = [0;0;-D(1)];
    r_i_im1(:,:,3) = [D(2);0;0];
    r_i_im1(:,:,4) = [0;D(3);-e2];
    r_i_im1(:,:,5) = [0;0;-D(4)];
    r_i_im1(:,:,6) = [0;-D(5);0];
    r_i_im1(:,:,7) = [0;0;D(6)];

%Convention of the used DH parameters
    Convention = 'Modified';

% Enter you number of DOF
    DOF = 6;
    
% Modification of angles from physical angles to algorithm angles.
    theta(:,1) = Theta_physalgo(Convention, AngleUnit,1,1,theta);
    
% Definition of the DH parameters
    dH = DH(Convention, 1);
    
% Trasformation Matrix between the world frame and the first DH frame
    TW0=[1   0   0   0;
         0   -1  0   0;
         0   0   -1  0;
         0   0   0   1];

% Creation of the Transform matrices and rotation matrices between links
    T = FKforDynmcs(Convention, DOF, dH, TW0, theta, AngleUnit);
    for index = 1:DOF
        R_im1_i(:,:,index) = T(1:3,1:3,index);
    end

%% Newton-Euler Iterative Dynamics Algorithm
% ----------------------------------------------------------------------------------------------------------------
% -------------------------------------------- Change this part --------------------------------------------------
% ----------------------------------------------------------------------------------------------------------------

% Parameters of the force and moments applied to the robot
% Add the parameters necessary for the robot to feel the effect of gravity
% while in free space without interacting with anything. 

    w0 = [0;0;0];                   % Angular velocity
    dw0 = [0;0;0];                  % Angular acceleration of the base
    dv0 = [0;0;0];                  % Gravity of the acceleration 
    fl = [0;0;0];                   % Force at the end effector
    nl = [0;0;0];                   % Moments at the end effector
    
% Build the outward iteration from joint 1 to joint 6 using R_im1_i as the
% Rotation matrix of (i-1)^R^i, r_i_im1 as the position vector of joint i
% from i-1 (r^(i/(i-1)), rcmi_i as the vector of the center of mass of joint
% i from the origin of joint i.  

% Angular velocity of joint 1 expressed in vector basis of joint 1
    w_i_i(:,:,1) = 0;
% Angular acceleration of joint 1 expressed in vector basis of joint 1
    dw_i_i(:,:,1) = 0; 
% Linear acceleration of joint 1 expressed in vector basis of joint 1
    dv_i_i(:,:,1) = 0;
% Linear acceleration of the center of mass of joint 1 expressed in vector basis of joint 1
    dv_cmi_i(:,:,1) = 0;
% Newton's second law of motion for the first joint
    F_i(:,:,1) = 0;
% Euler equation for joint 1
    M_i(:,:,1) = 0;

    for index = 2:6
    % Angular velocity of joint i expressed in vector basis of joint i
        w_i_i(:,:,index) = 0;
    % Angular acceleration of joint i expressed in vector basis of joint i
        dw_i_i(:,:,index) = 0;
    % Linear acceleration of joint i expressed in vector basis of joint i
        dv_i_i(:,:,index) = 0;
    % Linear acceleration of the center of mass of joint i expressed in vector basis of joint i
        dv_cmi_i(:,:,index) = 0;
    % Newton's second law of motion for the first joint
        F_i(:,:,index) = 0;
    % Euler equation for joint 1
        M_i(:,:,index) = 0;
    end

%Inward iteration from joint 6 to 1

% Force at the end effector 
    f_i(:,:,6) = fl + 0;
% Moment at the end  efector
    n_i(:,:,6) = 0 + nl;
% Torque of the 6th joint
    tau(6,1) = n_i(3,:,6);
    for index =5:-1:1
    % Force from the end effector at joint i
        f_i(:,:,index) = 0;
    % Moment form the end effector at joint i
        n_i(:,:,index) = 0;
    % Torque of the ith joint
        tau(index, 1) = transpose(n_i(3,:,index));
    end
end
