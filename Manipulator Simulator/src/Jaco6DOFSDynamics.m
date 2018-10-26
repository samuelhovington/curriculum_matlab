%Simon Michaud
%Creation: 2018-10-03
%Modifications 2018-10-09

%function tau = Jaco6DOFSDynamics(theta, dtheta, ddtheta)
clear all;
theta = [180,270,180,270,270,270];
dtheta = [0,0,0,0,0,0];
ddtheta = [0,0,0,0,0,0];
%% Masses et Inerties
g = 9.81;

m = [0.18198,0.42399,0.21100,0.09184, 0.09184,0.727];
Ipm = [0.0001020703 0.0003248936 0.0003536944 -5.214394e-9 -0.000021186541 -4.141348e-9];
Ic = [Ipm(2), Ipm(3), Ipm(1), -Ipm(6), -Ipm(4), Ipm(5)];
I(:,:,1) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ipm = [0.0037862305 0.0037251264 0.000069776668 0 0 5.903745e-8];
Ic = [Ipm(3), Ipm(2), Ipm(1), Ipm(6), Ipm(5), Ipm(4)];
I(:,:,2) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ipm = [0.0005109958 0.0004804467 0.00004810136 1.922692e-9 -0.000001586224 1.960234e-9];
Ic = [Ipm(2), Ipm(3), Ipm(1), -Ipm(6), -Ipm(4), Ipm(5)];
I(:,:,3) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ipm = [0.0001933181 0.00007567544 0.0001953757 -0.00003628615 -1.204e-8 1.37e-8];
Ic = [Ipm(3), -Ipm(1), Ipm(2), Ipm(5), -Ipm(6), -Ipm(4)];
I(:,:,4) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ic = [Ipm(3), Ipm(2), Ipm(1), Ipm(6), -Ipm(5), -Ipm(4)];
I(:,:,5) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ipm = [0.00004659624 0.00002647306 0.00005247987 0.00000413032 -1.0573e-7 -3.244e-8];
Ic = [Ipm(3), Ipm(1), Ipm(2), Ipm(5), -Ipm(6), -Ipm(4)];
I(:,:,6) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];

%% Variables pour la cinematique du robot    
q(1) = theta(1);
q(2) = theta(2)-90;
q(3) = theta(3)+90;
q(4) = -theta(4);
q(5) = -theta(5);
q(6) = theta(6);

D(1) = 0.2755;
D(2) = 0.4100;
D(3) = 0.2073;
D(4) = 0.1038;
D(5) = 0.1038;
D(6) = 0.16;
e2 = 0.0098;

%% Parametres DH du manipulateur
DH = [0, 0, -D(1), q(1); -pi/2, 0, 0, q(2); pi ,D(2), -e2, q(3); 3*pi/2, 0, (D(3)+D(4)), q(4); pi/2, 0, 0, q(5); 3*pi/2, 0, -(D(5)+D(6)), q(6)];

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

%% Creation des matrices de rotation et de transformation entre les frames
for i=1:6
    T(:,:,i)=[cosd(q(i)) -sind(q(i)) 0 a(i);...
        sind(q(i))*cos(alpha(i)) cos(alpha(i))*cosd(q(i)) -sin(alpha(i)) -d(i)*sin(alpha(i)); ...
        sind(q(i))*sin(alpha(i)) cosd(q(i))*sin(alpha(i)) cos(alpha(i)) d(i)*cos(alpha(i));...
        0 0 0 1];
    Ri_i1(:,:,i) = [cosd(q(i)) -sind(q(i)) 0;...
        sind(q(i))*cos(alpha(i)) cos(alpha(i))*cosd(q(i)) -sin(alpha(i)) ; ...
        sind(q(i))*sin(alpha(i)) cosd(q(i))*sin(alpha(i)) cos(alpha(i))];
    Ri1_i(:,:,i) = transpose(Ri_i1(:,:,i));
           
end
%% Creation des matrices de transformation, de rotation en fonction du frame de reference et vecteurs de position
T0=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];
T0(:,:,1) = T0 *T(:,:,1);
R0 = [1 0 0; 0 -1 0; 0 0 -1];
R0(:,:,1) = R0*Ri_i1(:,:,1);
Pi_i1(:,:,1) = [T0(1,4,1), T0(2,4,1), T0(3,4,1)];
for i = 2:6
    T0(:,:,i) = T0(:,:,i-1) *T(:,:,i);
    R0(:,:,i) = R0(:,:,i-1)*Ri_i1(:,:,i);
    Pi_i1(:,:,i) = [T0(1,4,i), T0(2,4,i), T0(3,4,i)];
    J(:,:,i) = Pi_i1(:,:,i);
end
J(:,:,1) = Pi_i1(:,:,1)/2;
J(:,:,4) = Pi_i1(:,:,3) +transpose(R0(:,:,3)*[0;D(3);0]);
J(:,:,6) = Pi_i1(:,:,4) + transpose(R0(:,:,5)*[0;-D(5);0]);
%% Vecteurs unitaires des differentes bases vectorielles
for i = 1:6
   ux(:,:,i) = R0(:,:,i)*[0.1;0;0];
   ux(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) +  ux(:,:,i)/sqrt(ux(1,1,i)^2 + ux(2,1,i)^2 + ux(3,1,i)^2);
   uy(:,:,i) = R0(:,:,i)*[0;0.1;0]; 
   uy(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + uy(:,:,i)/sqrt(uy(1,1,i)^2 + uy(2,1,i)^2 + uy(3,1,i)^2);
   uz(:,:,i) = R0(:,:,i)*[0;0;0.1]; 
   uz(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + uz(:,:,i)/sqrt(uz(1,1,i)^2 + uz(2,1,i)^2 + uz(3,1,i)^2);
end

%% Definition des centres de masses
Pi_cm(:,:,1) = J(:,:,1) + transpose(R0(:,:,1)*[-0.2371; 10.29; -69.17].*10^(-3));
Pi_cm(:,:,2) = J(:,:,2) + transpose(R0(:,:,2)*[205.2; -0.026; -22.3].*10^(-3));
Pi_cm(:,:,3) = J(:,:,3) + transpose(R0(:,:,3)*[0.071; 64.37; -19.569].*10^(-3));
Pi_cm(:,:,4) = J(:,:,4) + transpose(R0(:,:,4)*[0.01; 13.28; 68.52].*10^(-3));
Pi_cm(:,:,5) = J(:,:,6) + transpose(R0(:,:,5)*[-0.01; 68.52; 13.28].*10^(-3));
Pi_cm(:,:,6) =  J(:,:,6) + transpose(R0(:,:,6)*[0.03; 6.84; -82.22].*10^(-3));

%% Definition des fonctions
w0 = [0;0;0];
dw0 = [0;0;0];
dv0 = [0;0;g];

w(:,:,1) = Ri1_i(:,:,1)*w0 + dtheta(1)*uz(:,:,1);
dw(:,:,1) = Ri1_i(:,:,1)*dw0 + cross(Ri1_i(:,:,1)*w0,dtheta(1)*uz(:,:,1)) + ddtheta(1)*uz(:,:,1);
dv(:,:,1) = Ri1_i(:,:,1)*(cross(dw0, transpose(Pi_i1(:,:,1))) + cross(w0, cross(w0, transpose(Pi_i1(:,:,1))))+dv0);
dvcm(:,:,1) = cross(dw(:,:,1),transpose(Pi_cm(:,:,1)))+ cross(w(:,:,1), cross(w(:,:,1),transpose(Pi_cm(:,:,1))))+dv(:,:,1);
F(:,:,1) = m(1)*dvcm(:,:,1);
N(:,:,1) = I(:,:,1)*dw(:,:,1)+cross(w(:,:,1.3), I(:,:,1)*w(:,:,1));

for i=1:5
    w(:,:,i+1) = Ri1_i(:,:,i)*w(:,:,i) + dtheta(i+1)*uz(:,:,i+1);
    dw(:,:,i+1) = Ri1_i(:,:,i)*dw(:,:,i) + cross(Ri1_i(:,:,i)*w(:,:,i), dtheta(i+1)*uz(:,:,i+1)) +  ddtheta(i+1)*uz(:,:,i+1);
    dv(:,:,i+1) = Ri1_i(:,:,i)*(cross(dw(:,:,i), transpose(Pi_i1(:,:,i+1))) + cross(w(:,:,i), cross(w(:,:,i), transpose(Pi_i1(:,:,i+1))))+dv(:,:,i));
    dvcm(:,:,i+1) = cross(dw(:,:,i+1),transpose(Pi_cm(:,:,i+1)))+ cross(w(:,:,i+1), cross(w(:,:,i+1),transpose(Pi_cm(:,:,i+1))))+dv(:,:,i+1);
    F(:,:,i+1) = m(i)*dvcm(:,:,i+1);
    N(:,:,i+1) = I(:,:,i+1)*dw(:,:,i+1)+cross(w(:,:,i+1), I(:,:,i+1)*w(:,:,i+1));
end

f(:,:,6) = Ri_i1(:,:,6)*[0;0;0] + F(:,:,6);
n(:,:,6) = N(:,:,6) + Ri_i1(:,:,6)*[0;0;0] + cross(transpose(Pi_cm(:,:,6)), F(:,:,6));
nT(:,:,6) = transpose(n(:,:,6));
tau(6) = nT(1,3,6);
for i =5:-1:1
    f(:,:,i) = Ri_i1(:,:,i)*f(:,:,i+1) + F(:,:,i);
    n(:,:,i) = N(:,:,i) + Ri_i1(:,:,i)*n(:,:,i+1) + cross(transpose(Pi_cm(:,:,i)), F(:,:,i));
    nT(:,:,i) = transpose(n(:,:,i));
    tau(i) = nT(1,3,i);
end
%% plot 
figure(1)
plot3([ 0 T0(1,4,1) T0(1,4,2) T0(1,4,3) T0(1,4,4) T0(1,4,5) T0(1,4,6)],[0  T0(2,4,1) T0(2,4,2) T0(2,4,3) T0(2,4,4) T0(2,4,5) T0(2,4,6)],[0 T0(3,4,1) T0(3,4,2) T0(3,4,3) T0(3,4,4) T0(3,4,5) T0(3,4,6)], 'g')
 xlabel('X')
   ylabel('Y')
   xlim([-1 1])
   ylim([-1 1])
   zlim([-0.1 1.3])
   view(210,40)
grid on
hold on 

plot3([0],[0],[0], '-r*')
for i=1:6
   plot3([J(1,1,i)], [J(1,2,i)], [J(1,3,i)], '-o') 
end

% Graphiques des centres de masses,et des bases vectorielles
for i=1:6
%    plot3([Pi_i1(1,1,i) ux(1,1,i)], [Pi_i1(1,2,i) ux(2,1,i)], [Pi_i1(1,3,i) ux(3,1,i)], 'r')
%    plot3([Pi_i1(1,1,i) uy(1,1,i)], [Pi_i1(1,2,i) uy(2,1,i)], [Pi_i1(1,3,i) uy(3,1,i)], 'm')
%    plot3([Pi_i1(1,1,i) uz(1,1,i)], [Pi_i1(1,2,i) uz(2,1,i)], [Pi_i1(1,3,i) uz(3,1,i)], 'b')
%    plot3([Pi_cm(1,1,i)], [Pi_cm(1,2,i)], [Pi_cm(1,3,i)], '-x')
end

