%Jaco 6 DOF Spherical wrist, function that executes the inverse kinematics
%for this Kinova product. It takes in parameters the convention, the number
%of DOF, the DH parameters [a,d,alpha], the corresponding transformation 
%matrix from base to frame 0,the guessed angles of the DH algorithm of the
%joints and the desired cartesian position

%Created 2018-10-25 by Simon Michaud @Kinova
%Modified 2018-10-30

clear all;
convention = 'Classic';
DOF = 6;
theta = [-75, 163,45,266,258,0];
Pgoal = [0.2114; -0.2652; 0.5062; 1.6499; 1.1089; 0.1259];
TW0 = [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];
%% Parameters of the robot   
% Lengths of the links of the robots
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;

%Angles of the guessed position in the DH algorithm form
q = transpose(ThetaAlgo(theta));
angleUnit = 'Radians';

%DH parameters in the smae format as we have seen before [alpha, d, a]
DH = [pi/2, 0, -D1; pi, D2, 0; pi/2, 0, -e2; pi/2, 0, -(D3+D4); pi/2, 0, 0; pi, 0,  -(D5+D6)];
%Use of the Foward Kinematics to find the cartesian position of the guessed
%position
Pcurr = FKforIK(convention, DOF, DH, TW0,q, angleUnit);


%% Inverse Kinematics

Rgoal=EulerXYZtoRot(Pgoal(4:6));
ii = 1;
Err = abs(Pgoal-Pcurr);
while max(Err)>0.0001
% for ii=1:100                      %Uncomment line if you prefer a number of iterations rather than an error to respect

    Rcurr=EulerXYZtoRot(Pcurr(4:6));%Current rotational matrix from Euler angles
    delta_R=Rgoal-Rcurr;            %Difference between matrices
    Omega=delta_R/Rcurr;            %matrix differential rule: delta_R=Omega*Rcurr. So Omega = delta_R*inv(Rcurr)
    
    dX(4,:)=(Omega(3,2)-Omega(2,3))/2;  %Finding wx from W
    dX(5,:)=(Omega(1,3)-Omega(3,1))/2;  %Finding wy from W
    dX(6,:)=(Omega(2,1)-Omega(1,2))/2;  %Finding wz from W
    
    dX(1:3,:) = Pgoal(1:3)-Pcurr(1:3);  %positional displacement as seen in the class notes
    J = Jacobian_complete(convention, DOF, DH, TW0,q, angleUnit);   %Determination of the Jacobian at angle q
    dTheta = J\dX;                  %Inverse Kinematics formula
    q = q + dTheta                 %Finding new angles
    Pcurr = FKforIK(convention, DOF, DH, TW0, q, angleUnit); %Determining new Pcurr from new q
    Err = Pgoal-Pcurr;              %Determining Err from new Pcurr
    ii = ii+1;
    

end


function theta = ThetaAlgo(theta)
Deg2Rad = pi/180;
theta(1) = Deg2Rad * (theta(1) + 180);
theta(2) = Deg2Rad * (theta(2) +90);
theta(3) = Deg2Rad * (theta(3) + 90);
theta(4) = Deg2Rad * theta(4);
theta(5) = Deg2Rad * theta(5);
theta(6) = Deg2Rad * (theta(6) - 90);
end
