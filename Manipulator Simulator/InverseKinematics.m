%Jaco 6 DOF Spherical wrist, function that executes the inverse kinematics
%for a this kinova product
%Created 2018-10-25 by Simon Michaud @Kinova
%Modified 

clear all;
convention = 'modified';
DOF = 6;
theta = [235.15,187.47,101.8,276.27,235.13,0];
%% Foward kinematics fo the robot   
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;

q = ThetaAlgo(theta);

DH = [0, 0, -D1, q(1); pi/2, 0, 0, q(2); pi ,D2, -e2, q(3); pi/2, 0, -(D3+D4), q(4); pi/2, 0, 0, q(5); pi/2, 0, (D5+D6), q(6)];
TW0 = [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];
Pcurr = FKforIK(DOF, convention, DH, TW0,q);
% 
% for i= 1:DOF
%    R0(:,:,i) = [T0(1,1,i), T0(1,2,i), T0(1,3,i); T0(2,1,i), T0(2,2,i), T0(2,3,i);T0(3,1,i), T0(3,2,i), T0(3,3,i)];
%    ri_0(:,:,i) = [T0(1,4,i);T0(2,4,i);T0(3,4,i)];
% end


%% Definition of the jacobian
J = Jacobian(DOF, DH, TW0);

%% Inverse Kinematics
Pgoal = [0.5;-0.25;0.7;-2;1.5;1.9];

dX = Pgoal-Pcurr;
dTheta = inv(J)*dX;
q = transpose(q) + dTheta;
Pcurr = FKforIK(DOF, convention, DH, TW0, q);
dX = Pgoal-Pcurr;
j = 1;

% while max(Err)>0.001

for i=1:100
    dX = Pgoal-Pcurr;
    dTheta = transpose(J)*dX;
    q = transpose(q) + dTheta;
    Pcurr = FKforIK(DOF, convention, DH, TW0,q)
    j = j+1
end

