%Jaco 6 DOF Spherical wrist, function that executes the inverse kinematics
%for this Kinova product. It takes in parameters the convention, the number
%of DOF, the DH parameters [a,d,alpha], the corresponding transformation 
%matrix from base to frame 0, the desired cartesian position,the guessed 
%angles of the DH algorithm of the joints and unit of the guessed angular
%position (radians or degrees)
%Created 2018-10-25 by Simon Michaud @Kinova
%Modified 2018-10-30

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
        
        
% The rest of the file should not be modified
        J = Jacobian_complete(convention, DOF, DH, TW0,q, angleUnit);   %Determination of the Jacobian at angle q
        dTheta = 0;                         %Inverse Kinematics formula
        q = q + dTheta;                     %Finding new angles
        Pcurr = FKforIK(convention, DOF, DH, TW0, q, angleUnit); %Determining new Pcurr from new q
        Err = abs(Pcurr_old-Pcurr);         %Determining Err from new Pcurr
        ii = ii+1;
        
    end
    
end

