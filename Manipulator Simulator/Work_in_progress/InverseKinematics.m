%Jaco 6 DOF Spherical wrist, function that executes the inverse kinematics
%for this Kinova product
%Created 2018-10-25 by Simon Michaud @Kinova
%Modified 2018-10-29

clear all;
convention = 'classic';
DOF = 6;
theta = [230.15,186.47,100.8,275.27,234.13,0];
%% Forward kinematics fo the robot   
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;

q(:,:,1) = transpose(ThetaAlgo(theta));

DH = [0 -D1 pi/2; D2 0 pi; 0 -e2 pi/2; 0 -(D3 + D4) pi/2; 0 0 pi/2; 0 -(D5+D6) pi];
TW0 = [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];
Pcurr(:,:,1) = FKforIK(DOF, convention, DH, TW0,q)
% 
% for i= 1:DOF
%    R0(:,:,i) = [T0(1,1,i), T0(1,2,i), T0(1,3,i); T0(2,1,i), T0(2,2,i), T0(2,3,i);T0(3,1,i), T0(3,2,i), T0(3,3,i)];
%    ri_0(:,:,i) = [T0(1,4,i);T0(2,4,i);T0(3,4,i)];
% end

%% Inverse Kinematics
Pgoal = [0.5;-0.25;0.7;-2;1.5;1.9];
P = FKforIK(DOF, convention, DH, TW0, q);

%     dX(:,:,1) = Pgoal-Pcurr(:,:,1);
%     J = Jacobian(DOF, DH, TW0,q(:,:,1));
%     dTheta(:,:,1) = J\dX(:,:,1);
%     q(:,:,2) = q(:,:,1) - dTheta(:,:,1);
%     Pcurr(:,:,2) = FKforIK(DOF, convention, DH, TW0,q(:,:,2))
j = 1;

% while max(Err)>0.001
for ii=1:100
    dX(1:3,:,ii) = Pgoal(1:3)-Pcurr(1:3,:,ii)
    Rgoal=EulerXYZtoRot(Pgoal(4:6))
    Rcurr=EulerXYZtoRot(Pcurr(4:6,:,ii));
    %matrix differential rule: delta_R=Omega*Rcurr. So Omega =
    %delta_R*inv(Rcurr)
    delta_R=Rgoal-Rcurr;
    Omega=delta_R/Rcurr; %equivalent in Matlab to delta_R* inv(Rcurr);
    dX(4,:,ii)=(Omega(2,3)-Omega(3,2))/2;
    dX(5,:,ii)=(Omega(3,1)-Omega(1,3))/2;
    dX(6,:,ii)=(Omega(1,2)-Omega(2,1))/2;
    dX
    J = Jacobian(DOF, DH, TW0,q(:,:,ii))
    dTheta(:,:,ii) = inv(J)*dX(:,:,ii)
    q(:,:,ii+1) = q(:,:,ii) + dTheta(:,:,ii);
    Pcurr(:,:,ii+1) = FKforIK(DOF, convention, DH, TW0,q(:,:,ii+1))
    j = j+1;
end

