%Created on 2018-11-15 by Simon Michaud @Kinova
%Modified on 2018-11-20
%Script that executes the dynamics of a 2 DOF robot with the same 2
%actuators position as joint 1 and 2 of Jaco 
clear all; close all;
theta = [180+180,270+90,180,270,270,270];
dtheta = [3;9];
ddtheta = [5;7];
theta = theta*pi/180;

m1 = 0.18;
m2 = 0.42;
D1 = 0.2753;
D2 = 0.41;
g = 9.81;
Ixx1 = 0;
Iyy1 = 0;
Izz1 = 0;
Ixy1 = 0;
Ixz1 = 0;
Iyz1 = 0;
Icm1 = [Ixx1 Ixy1 Ixz1; Ixy1 Iyy1 Iyz1; Ixz1 Iyz1 Izz1];
Ixx2 = 0;
Iyy2 = 0;
Izz2 = 0;
Ixy2 = 0;
Ixz2 = 0;
Iyz2 = 0;
Icm2 = [Ixx2 Ixy2 Ixz2; Ixy2 Iyy2 Iyz2; Ixz2 Iyz2 Izz2];
f3 = [0;0;0];
n3 = [0;0;0];
rcm1_1 = [0;0;D1/2];
rcm2_2 = [D2/2;0;0];
r_1_0 = [0;0;0];
r_2_1 = [D1;0;0];
w0 = [0;0;0];
dw0 = [0;0;0];
dv0 = [0;0;-g];

R_0_1 = [cos(theta(1)), -sin(theta(1)), 0; sin(theta(1)), cos(theta(1)), 0;0,0,1];
R_1_2 = [-sin(theta(2)), cos(theta(2)), 0;0,0,-1; -cos(theta(2)), -sin(theta(2)), 0];

w_1_1 = [0;0;dtheta(1)];
w_1_1v = transpose(R_0_1) * w0 + [0;0;dtheta(1)];

w_2_2 = [-dtheta(1)*cos(theta(2)); -dtheta(1)*sin(theta(2)); dtheta(2)];
w_2_2v = transpose(R_1_2) * w_1_1v + [0;0;dtheta(2)];

dw_1_1 = [0;0;ddtheta(1)];
dw_1_1v = transpose(R_0_1)*dw0 + cross(transpose(R_0_1)*w0,[0;0;dtheta(1)]) + [0;0;ddtheta(1)]; 

dw_2_2 = [-ddtheta(1)*cos(theta(2)) - dtheta(1)*dtheta(2)*sin(theta(2));
    -ddtheta(1)*sin(theta(2))+ dtheta(1)*dtheta(2)*cos(theta(2));
    ddtheta(2)];
dw_2_2v = transpose(R_1_2)*dw_1_1 + cross(transpose(R_1_2)*w_1_1,[0;0;dtheta(2)]) + [0;0;ddtheta(2)];

dv_1_1 = [0;0;-g];
dv_1_1v = transpose(R_0_1)*(cross(dw0, r_1_0)+cross(w0,cross(w0,r_1_0)) + dv0);

dv_2_2 = [-D1*dtheta(1)^2*(sin(theta(2)))^2 + g*cos(theta(2));
    D1*dtheta(1)^2*sin(theta(2))*cos(theta(2)) + g*sin(theta(2));
    D1*sin(theta(2))*ddtheta(1)];
dv_2_2v = transpose(R_1_2)*(cross(dw_1_1v, R_1_2*r_2_1)+cross(w_1_1v,cross(w_1_1v, R_1_2*r_2_1)) + dv_1_1v);

dv_cm1_1 = [-ddtheta(1)*rcm1_1(2) - dtheta(1)^2*rcm1_1(1); 
    ddtheta(1)*rcm1_1(1) + dtheta(1)^2*rcm1_1(2); -g];
dv_cm1_1v = cross(dw_1_1v, rcm1_1) + cross(w_1_1v, cross(w_1_1v,rcm1_1)) + dv_1_1v;

dv_cm2_2 = [(-ddtheta(1)*sin(theta(2)) + dtheta(1)*dtheta(2)*cos(theta(2)))*rcm2_2(3) + ddtheta(2)*rcm2_2(2) + (-dtheta(1)*sin(theta(2)))*(-dtheta(1)*cos(theta(2))*rcm2_2(2) + dtheta(1)*sin(theta(2))*rcm2_2(1)) - dtheta(2)*(dtheta(1)*cos(theta(2))*rcm2_2(3) + dtheta(2)*rcm2_2(1))-D1*dtheta(1)^2*(sin(theta(2)))^2 + g*cos(theta(2));
    -(-ddtheta(1)*cos(theta(2)) - dtheta(1)*dtheta(2)*sin(theta(2)))*rcm2_2(3) + ddtheta(2)*rcm2_2(1) + (dtheta(1)*cos(theta(2)))*(-dtheta(1)*cos(theta(2))*rcm2_2(2) + dtheta(1)*sin(theta(2))*rcm2_2(1)) + dtheta(2)*(-dtheta(1)*sin(theta(2))*rcm2_2(3)-dtheta(2)*rcm2_2(2)) + D1*dtheta(1)^2*sin(theta(2))*cos(theta(2)) + g*sin(theta(2));
    (-ddtheta(1)*cos(theta(2))-dtheta(1)*dtheta(2)*sin(theta(2)))*rcm2_2(2) - (-ddtheta(1)*sin(theta(2)) + dtheta(1)*dtheta(2)*cos(theta(2)))*rcm2_2(1) + (-dtheta(1)*cos(theta(2)))*(dtheta(1)*cos(theta(2))*rcm2_2(3)+dtheta(2)*rcm2_2(1)) + (dtheta(1)*sin(theta(2)))*(-dtheta(1)*sin(theta(2))*rcm2_2(3) - dtheta(2)*rcm2_2(2)) + D1*sin(theta(2))*ddtheta(1)];
dv_cm2_2v = cross(dw_2_2v, rcm2_2) + cross(w_2_2v, cross(w_2_2v,rcm2_2)) + dv_2_2v;

F_1 = [m1*(-ddtheta(1)*rcm1_1(2) - dtheta(1)^2*rcm1_1(1));m1*(ddtheta(1)*rcm1_1(1) - dtheta(1)^2*rcm1_1(2));-m1*g];
F_1v = m1*dv_cm1_1v;

F_2v = m2*dv_cm2_2v;

M_1  = [Ixz1*ddtheta(1) - Iyz1*dtheta(1)^2; Iyz1*ddtheta(1) + Ixz1*dtheta(1)^2; Izz1*ddtheta(1)];
M_1v = Icm1*dw_1_1 + cross(w_1_1, Icm1*w_1_1);

M_2 = [Ixx2*(-ddtheta(1)*cos(theta(2))-dtheta(1)*dtheta(2)*sin(theta(2))) + Ixy2*(-ddtheta(1)*sin(theta(2)) + dtheta(1)*dtheta(2)*cos(theta(2))) + Ixz2*ddtheta(2) + (-dtheta(1)*sin(theta(2)))*(-Ixz2*dtheta(1)*cos(theta(2)) - Iyz2*dtheta(1)*sin(theta(2)) + Izz2*dtheta(2)) - dtheta(2)*(-Ixy2*dtheta(1)*cos(theta(2)) - Iyy2*dtheta(1)*sin(theta(2)) + Iyz2*dtheta(2));
    Ixy2*(-ddtheta(1)*cos(theta(2)) - dtheta(1)*dtheta(2)*sin(theta(2))) + Iyy2*(-ddtheta(1)*sin(theta(2)) + dtheta(1)*dtheta(2)*cos(theta(2))) + Iyz2*ddtheta(2) + (dtheta(1)*cos(theta(2)))*(-Ixz2*dtheta(1)*cos(theta(2)) - Iyz2*dtheta(1)*sin(theta(2)) + Izz2*dtheta(2)) + dtheta(2)*(-Ixx2*dtheta(1)*cos(theta(2)) - Ixy2*dtheta(1)*sin(theta(2)) + Ixz2*dtheta(2));
    Ixz2*(-ddtheta(1)*cos(theta(2)) - dtheta(1)*dtheta(2)*sin(theta(2))) + Iyz2*(-ddtheta(1)*sin(theta(2)) + dtheta(1)*dtheta(2)*cos(theta(2))) + Izz2*ddtheta(2) + (-dtheta(1)*cos(theta(2)))*(-Ixy2*dtheta(1)*cos(theta(2)) - Iyy2*dtheta(1)*sin(theta(2)) + Iyz2*dtheta(2)) + (dtheta(1)*sin(theta(2)))*(-Ixx2*dtheta(1)*cos(theta(2)) - Ixy2*dtheta(1)*sin(theta(2)) + Ixz2*dtheta(2))];
M_2v = Icm2*dw_2_2 + cross(w_2_2, Icm2*w_2_2);

f_2 = m2*dv_cm2_2;
f_2v = f3 + F_2v;

f_1 = [m2*(dv_cm2_2(1)*(-sin(theta(2))) + dv_cm2_2(2)*cos(theta(2))) + m1*(-ddtheta(1)*rcm1_1(2) - dtheta(1)^2*rcm1_1(1));
    m2*(-dv_cm2_2(3)) + m1*(ddtheta(1)*rcm1_1(1) - dtheta(1)^2*rcm1_1(2));
    m2*(-dv_cm2_2(1)*cos(theta(2)) - dv_cm2_2(2)*sin(theta(2))) - m1*g];

f_1v = R_1_2*f_2v  +F_1v;

n_2 = [M_2(1) + m2*dv_cm2_2(3)*rcm2_2(2) - m2*dv_cm2_2(2)*rcm2_2(3);
    M_2(2) - m2*dv_cm2_2(3)*rcm2_2(1) - m2*dv_cm2_2(1)*rcm2_2(3);
    M_2(3) + m2*dv_cm2_2(2)*rcm2_2(1) - m2*dv_cm2_2(1)*rcm2_2(2)];
n_2v = M_2 + n3 + cross(rcm2_2, F_2v) + f3;

n_1x = M_1(1) - M_2(1)*sin(theta(2)) - m2*dv_cm2_2(3)*rcm2_2(2)*sin(theta(2))+ m2*dv_cm2_2(2)*sin(theta(2))*rcm2_2(3) + M_2(2)*cos(theta(2)) - m2*dv_cm2_2(3)*rcm2_2(1)*cos(theta(2)) + m2*dv_cm2_2(1)*rcm2_2(3)*cos(theta(2)) - m1*rcm1_1(3)*(ddtheta(1)*rcm1_1(1) - dtheta(1)^2*rcm1_1(2)) -D1*cos(theta(2))*m2*dv_cm2_2(3);
n_1y = M_1(2) - M_2(3) - m2*dv_cm2_2(2)*rcm2_2(1) - m2*dv_cm2_2(1)*rcm2_2(2) + rcm1_1(3)*m1*(-ddtheta(1)*rcm1_1(2) + (dtheta(1)^2)*rcm1_1(1)) + D1*sin(theta(2))*(-m2*dv_cm2_2(1)*cos(theta(2)) - m2*dv_cm2_2(2)*sin(theta(2))) + (D1*cos(theta(2)))*(-m2*dv_cm2_2(1)*sin(theta(2)) + m2*dv_cm2_2(2)*cos(theta(2)));
n_1z = M_1(3) - M_2(1)*cos(theta(2)) - m2*dv_cm2_2(3)*rcm2_2(2)*cos(theta(2)) + m2*dv_cm2_2(2)*cos(theta(2))*rcm2_2(3) - M_2(2)*sin(theta(2)) + m2*dv_cm2_2(3)*rcm2_2(1)*sin(theta(2)) - m2*dv_cm2_2(1)*rcm2_2(3)*sin(theta(2)) + rcm1_1(1)*m1*(ddtheta(1)*rcm1_1(1) - dtheta(1)^2*rcm1_1(2)) - rcm1_1(2)*m1*(-ddtheta(1)*rcm1_1(2) + dtheta(1)^2*rcm1_1(1))+D1*sin(theta(2))*m2*dv_cm2_2(3);
    

n_1 = [n_1x; n_1y; n_1z];
n_1v = M_1 + R_1_2*n_2 + cross(rcm1_1, F_1) + cross(r_2_1, R_1_2*f_2);



tau = [n_1(3), n_2(3)]
tau_v = [n_1v(3), n_2v(3)]