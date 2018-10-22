%Simon Michaud
%Creation: 2018-10-05
%Modifications 2018-10-22

clear all;
convention = 'modified';
DOF = 6;
theta = [180,270,90,270,270,270];
% theta = [180, 180,180,180,180,180];
% theta = [238,238,100,210,200,0];
%% Variables pour la cinematique du robot    
q(1) = theta(1)+180;
q(2) = theta(2)+90;
q(3) = theta(3)+90;
q(4) = theta(4);
q(5) = theta(5)+180;
q(6) = -(theta(6)+90);


D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;

%% Parametres DH du manipulateur
DH = [0, 0, -D1, q(1); pi/2, 0, 0, q(2); pi ,D2, -e2, q(3); pi/2, 0, -(D3+D4), q(4); pi/2, 0, 0, q(5); pi/2, 0, (D5+D6), q(6)];
 
for i=1:DOF
       alpha(i) = DH(i,1);
       d(i) = DH(i,3);
       a(i) = DH(i,2);
   end    
 

%% Creation des matrices de rotation et de transformation entre les frames
for i=1:DOF
    if strcmp(convention, 'classic')
        T(:,:,i)=[cosd(q(i)) -sind(q(i))*cos(alpha(i)) sind(q(i))*sin(alpha(i)) a(i)*cosd(q(i));...
                sind(q(i)) cos(alpha(i))*cosd(q(i)) -cosd(q(i))*sin(alpha(i)) a(i)*sind(q(i)); ...
                0 sin(alpha(i)) cos(alpha(i)) d(i);...
                0 0 0 1];
    elseif strcmp(convention, 'modified')
    T(:,:,i)=[cosd(q(i)) -sind(q(i)) 0 a(i);...
        sind(q(i))*cos(alpha(i)) cos(alpha(i))*cosd(q(i)) -sin(alpha(i)) -d(i)*sin(alpha(i)); ...
        sind(q(i))*sin(alpha(i)) cosd(q(i))*sin(alpha(i)) cos(alpha(i)) d(i)*cos(alpha(i));...
        0 0 0 1];
    end
    Ri_i1(:,:,i) = [cosd(q(i)) -sind(q(i)) 0;...
        sind(q(i))*cos(alpha(i)) cos(alpha(i))*cosd(q(i)) -sin(alpha(i)) ; ...
        sind(q(i))*sin(alpha(i)) cosd(q(i))*sin(alpha(i)) cos(alpha(i))];
    Ri1_i(:,:,i) = transpose(Ri_i1(:,:,i));
           
end
%% Creation des matrices de transformation et de rotation en fonction du frame de reference
T0=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];
T0(:,:,1) = T0 *T(:,:,1);
R0 = [1 0 0; 0 -1 0; 0 0 -1];
R0(:,:,1) = R0*Ri_i1(:,:,1);
Pi_i1(:,:,1) = [T0(1,4,1), T0(2,4,1), T0(3,4,1)];
for i = 2:DOF
    T0(:,:,i) = T0(:,:,i-1) *T(:,:,i);
    R0(:,:,i) = R0(:,:,i-1)*Ri_i1(:,:,i);
    Pi_i1(:,:,i) = [T0(1,4,i), T0(2,4,i), T0(3,4,i)];
end
%% Vecteurs unitaires des differentes bases vectorielles
for i = 1:DOF
   ux(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + R0(:,:,i)*[0.1;0;0]; 
   uy(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + R0(:,:,i)*[0;0.1;0]; 
   uz(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + R0(:,:,i)*[0;0;0.1]; 
end

%% plot 
figure(1)
plot3([ 0 T0(1,4,1) T0(1,4,2) T0(1,4,3) T0(1,4,4) T0(1,4,5) T0(1,4,6)],[0  T0(2,4,1) T0(2,4,2) T0(2,4,3) T0(2,4,4) T0(2,4,5) T0(2,4,6)],[0 T0(3,4,1) T0(3,4,2) T0(3,4,3) T0(3,4,4) T0(3,4,5) T0(3,4,6)], 'g')
  
xlabel('X')
   ylabel('Y')
%    xlim([-1 1])
%    ylim([-1 1])
%    zlim([-0.1 1.3])
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