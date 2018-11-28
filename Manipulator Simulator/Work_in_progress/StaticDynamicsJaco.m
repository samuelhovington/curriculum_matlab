%Simon Michaud
%Creation: 2018-11-08
%Modifications 2018-11-08
%Statics of a Jaco robot using a simple method of the Jacobian and a
%simplification of teh gravity force
clear all;
theta = [180;270;180;270;270;270];

%% Masses and Inertias
g = 9.81;

m = [0.18198,0.42399,0.21100,0.09184, 0.09184,0.727];
I = Inertia();

%% DH Parameters 
convention = 'Modified';
angleUnit = 'Degrees';

DH = DH(convention, 1);
q = Theta_physalgo(convention,angleUnit,1,1,theta);

T_0_W=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];

%% Transformation and Rotation Matrices 
T_im1_i = Transformi_i1(convention,angleUnit,DH,q);
T0(:,:,1) = T_0_W*T_im1_i(:,:,1);
for index = 2:6
   T0(:,:,index) = T0(:,:,index-1)*T_im1_i(:,:,index);
end
for index = 1:6
   R_im1_i(:,:,index) = T_im1_i(1:3,1:3,index);
   R0(:,:,index) = T0(1:3,1:3,index);
   Pi_i1(:,:,index) = [T0(1,4,index), T0(2,4,index), T0(3,4,index)];

end
angleUnit = 'Radians';
for i = 1:6
   ux(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + R0(:,:,i)*[0.1;0;0]; 
   uy(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + R0(:,:,i)*[0;0.1;0]; 
   uz(:,:,i) = transpose([T0(1,4,i), T0(2,4,i), T0(3,4,i)]) + R0(:,:,i)*[0;0;0.1]; 
end

%% Statics using the Jacobian
J = Jacobian_complete(convention, 6,DH, T_0_W, q, angleUnit);
M = sum(m);
F = R0(:,:,6)*[0;0;-9.81*3.6];
F = [F(1:3,1);0;0;0];
tau = transpose(J)*F;

%% plot 
% figure(1)
% plot3([ 0 T0(1,4,1) T0(1,4,2) T0(1,4,3) T0(1,4,4) T0(1,4,5) T0(1,4,6)],[0  T0(2,4,1) T0(2,4,2) T0(2,4,3) T0(2,4,4) T0(2,4,5) T0(2,4,6)],[0 T0(3,4,1) T0(3,4,2) T0(3,4,3) T0(3,4,4) T0(3,4,5) T0(3,4,6)], 'g')
%  xlabel('X')
%    ylabel('Y')
%    xlim([-1 1])
%    ylim([-1 1])
%    zlim([-0.1 1.3])
%    view(210,40)
% grid on
% hold on 
% 
% plot3([0],[0],[0], '-r*')
% for i=1:6
%    plot3([Pi_i1(1,1,i)], [Pi_i1(1,2,i)], [Pi_i1(1,3,i)], '-o') 
%    plot3([Pi_i1(1,1,i) ux(1,1,i)], [Pi_i1(1,2,i) ux(2,1,i)], [Pi_i1(1,3,i) ux(3,1,i)], 'r')
%    plot3([Pi_i1(1,1,i) uy(1,1,i)], [Pi_i1(1,2,i) uy(2,1,i)], [Pi_i1(1,3,i) uy(3,1,i)], 'm')
%    plot3([Pi_i1(1,1,i) uz(1,1,i)], [Pi_i1(1,2,i) uz(2,1,i)], [Pi_i1(1,3,i) uz(3,1,i)], 'b')
% end