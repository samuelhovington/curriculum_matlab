%Created 2018-10-25 by Simon Michaud @Kinova
%Modified 2018-11-06

%Inverse kinematics function that returns the angular position of the robot
%from a given cartesian position 

%Arguments: Convention of the DH parameters, number of degrees of freedom,
            %the DH parameters, the transformation matrix from world frame
            %to the 0 DH frame, the goal cartesian position of the end 
            %effector, the guessed angular position and the units of the 
            %angles of the angular position
%Returns:   6x1 vector containing the angular position of the robot

function q = InverseKinematics_to_complete(convention, DOF,DH, TW0, Pgoal, theta, angleUnit)
%% Foward Kinematics for initial point
%Obtainning right angles for the different functions
    q = theta;
    if strcmp(angleUnit, 'Degrees')
        q = q*pi/180;
        angleUnit = 'Radians';
    end
    
%Use of the Foward Kinematics to find the cartesian position of the guessed
%position
    Pcurr = FKforIK(convention, DOF, DH, TW0, q, angleUnit);


%% Inverse Kinematics algorithm using Newton-Raphson Iterative Method for multivariable functions

    Rgoal=EulerXYZtoRot(Pgoal(4:6));        %Goal rotation matrix
    ii = 1;
    Err = abs(Pgoal-Pcurr);                 %Error between Pgoal and Pcurr
    
    while max(Err)>0.001                    %Beginning of the iterative method
%     for ii=1:1000                         %Uncomment line if you prefer a number of iterations rather than an error to respect
        Pcurr_old = Pcurr;                  %Necessary to determine the error between iterations
        Rcurr=EulerXYZtoRot(Pcurr(4:6));    %Current rotational matrix from Euler angles
        delta_R=Rgoal-Rcurr;                %Difference between matrices
        
% ----------------------------------------------------------------------------------------------------------------
% ------------------------------- Generating the inverse kinematics algorithm --------------------------------------
% ----------------------------------------------------------------------------------------------------------------
        
        Omega=0;                %matrix differential rule

        dX(4,:)=0;              %Finding wx from Omega
        dX(5,:)=0;              %Finding wy from Omega
        dX(6,:)=0;              %Finding wz from Omega

        dX(1:3,:) = 0;  %positional displacement as seen in the class notes
        
        
%Add your Jacobian if it works and if it does not remove "_to" from the
%following line
        J = Jacobian_to_complete(convention, DOF, DH, TW0,q, angleUnit);   %Determination of the Jacobian at angle q
% The rest of the file should not be modified
        dTheta = 0;                         %Inverse Kinematics formula
        q = q + dTheta;                     %Finding new angles
        Pcurr = FKforIK(convention, DOF, DH, TW0, q, angleUnit); %Determining new Pcurr from new q
        Err = abs(Pcurr_old-Pcurr);         %Determining Err from new Pcurr
        ii = ii+1;
        
    end
    
end

