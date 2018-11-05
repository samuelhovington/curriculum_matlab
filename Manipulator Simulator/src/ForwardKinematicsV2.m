%Simon Michaud
%Creation: 2018-10-05
%Modifications

clear all;
theta = [180,270,90,270,270,270];
% theta = [180, 180,180,180,180,180];
% theta = [238,238,100,210,200,0];
% theta = [235.15,187.47,101.8,276.27,235.13,0];
% theta = (180/pi)*[-127.987677411609;3.44645346157629;-34.3632427667322;24.2451949058304;1.47126127733334;113.110658869994]
%% Variables pour la cinematique du robot    
q(1) = theta(1)+180;
q(2) = theta(2)+90;
q(3) = theta(3)+90;
q(4) = -theta(4);
q(5) = -theta(5);
q(6) = -theta(6)+90;


D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;

%% Parametres DH du manipulateur
DH = [0, 0, -D1, q(1); pi/2, 0, 0, q(2); pi ,D2, -e2, q(3); 3*pi/2, 0, (D3+D4), q(4); pi/2, 0, 0, q(5); pi/2, 0, (D5+D6), q(6)];

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
    Ri_i1(:,:,i) = [T(1:3,1,i), T(1:3,2,i), T(1:3,3,i)];
    Ri1_i(:,:,i) = transpose(Ri_i1(:,:,i));
           
end
%% Creation des matrices de transformation et de rotation en fonction du frame de reference
T0=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];
T0(:,:,1) = T0 *T(:,:,1);
R0 = [1 0 0; 0 -1 0; 0 0 -1];
R0(:,:,1) = R0*Ri_i1(:,:,1);
Pi_i1(:,:,1) = [T0(1,4,1), T0(2,4,1), T0(3,4,1)];
for i = 2:6
    T0(:,:,i) = T0(:,:,i-1) *T(:,:,i);
    R0(:,:,i) = [T0(1:3,1,i), T0(1:3,2,i), T0(1:3,3,i)];
    Pi_i1(:,:,i) = [T0(1,4,i), T0(2,4,i), T0(3,4,i)];
end
%% Vecteurs unitaires des differentes bases vectorielles
for i = 1:6
   ux(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + R0(:,:,i)*[0.1;0;0]; 
   uy(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + R0(:,:,i)*[0;0.1;0]; 
   uz(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + R0(:,:,i)*[0;0;0.1]; 
end
Eul = MatRotationToEuler(R0(:,:,6))
Rot = EulerXYZtoRot(Eul)
Rot2  =EulerXYZtoRot([2.98;1.52;-3.10])
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
   plot3([Pi_i1(1,1,i)], [Pi_i1(1,2,i)], [Pi_i1(1,3,i)], '-o') 
   plot3([Pi_i1(1,1,i) ux(1,1,i)], [Pi_i1(1,2,i) ux(2,1,i)], [Pi_i1(1,3,i) ux(3,1,i)], 'r')
   plot3([Pi_i1(1,1,i) uy(1,1,i)], [Pi_i1(1,2,i) uy(2,1,i)], [Pi_i1(1,3,i) uy(3,1,i)], 'm')
   plot3([Pi_i1(1,1,i) uz(1,1,i)], [Pi_i1(1,2,i) uz(2,1,i)], [Pi_i1(1,3,i) uz(3,1,i)], 'b')
end





