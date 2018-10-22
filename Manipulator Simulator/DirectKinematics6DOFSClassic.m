%function [T,T1,T2,T3,T4,T5,T6, T7, Jacob] = DirectKinematics7DOF(dH, theta)

theta = [180,270,90,270,270,270];
%dH=[0 -0.275500000000000 1.57079632679490;0 0 1.57079632679490;0 -0.410000000000000 1.57079632679490;0 -0.00980000000000000 1.57079632679490;0 -0.31150000000000000 1.57079632679490;0 0 1.57079632679490;0 -0.263750000000000 pi]
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;

e2 = 0.0098;
Deg2Rad = pi/180;

theta(1) = Deg2Rad * (theta(1) + 180);
theta(2) = Deg2Rad * (theta(2) - 90);
theta(3) = Deg2Rad * (theta(3) - 90);
theta(4) = Deg2Rad * theta(4);
theta(5) = Deg2Rad * theta(5);
theta(6) = Deg2Rad * (theta(6) + 90);
dH = [0 -D1 pi/2; -D2 0 pi; 0 -e2 pi/2; 0 -(D3 + D4) pi/2; 0 0 pi/2; 0 -(D5+D6) pi];
    

alpha(1)=dH(1,3);
alpha(2)=dH(2,3);
alpha(3)=dH(3,3);
alpha(4)=dH(4,3);
alpha(5)=dH(5,3);
alpha(6)=dH(6,3);
% alpha(7)=dH(7,3);

d(1)=dH(1,2);
d(2)=dH(2,2);
d(3)=dH(3,2);
d(4)=dH(4,2);
d(5)=dH(5,2);
d(6)=dH(6,2);
% d(7)=dH(7,2);

a(1)=dH(1,1);
a(2)=dH(2,1);
a(3)=dH(3,1);
a(4)=dH(4,1);
a(5)=dH(5,1);
a(6)=dH(6,1);
% a(7)=dH(7,1);

for i=1:6
    T(:,:,i)=[cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));...
        sin(theta(i)) cos(alpha(i))*cos(theta(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i)); ...
        0 sin(alpha(i)) cos(alpha(i)) d(i);...
        0 0 0 1];
end

T0=[1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1];
T1=T0*T(:,:,1);
T2=T1*T(:,:,2);
T3=T2*T(:,:,3);
T4=T3*T(:,:,4);
T5=T4*T(:,:,5);
T6=T5*T(:,:,6);
%T7=T6*T(:,:,7);

T=T6;

Jacobcol1=[cross([0;0;-1], [T6(1,4); T6(2,4); T6(3,4)]); 0; 0; -1];
Jacobcol2=[cross([T1(1,3); T1(2,3); T1(3,3)], [T6(1,4)-T1(1,4); T6(2,4)-T1(2,4); T6(3,4)-T1(3,4)]); T1(1,3); T1(2,3); T1(3,3)];
Jacobcol3=[cross([T2(1,3); T2(2,3); T2(3,3)], [T6(1,4)-T2(1,4); T6(2,4)-T2(2,4); T6(3,4)-T2(3,4)]); T2(1,3); T2(2,3); T2(3,3)];
Jacobcol4=[cross([T3(1,3); T3(2,3); T3(3,3)], [T6(1,4)-T3(1,4); T6(2,4)-T3(2,4); T6(3,4)-T3(3,4)]); T3(1,3); T3(2,3); T3(3,3)];
Jacobcol5=[cross([T4(1,3); T4(2,3); T4(3,3)], [T6(1,4)-T4(1,4); T6(2,4)-T4(2,4); T6(3,4)-T4(3,4)]); T4(1,3); T4(2,3); T4(3,3)];
Jacobcol6=[cross([T5(1,3); T5(2,3); T5(3,3)], [T6(1,4)-T5(1,4); T6(2,4)-T5(2,4); T6(3,4)-T5(3,4)]); T5(1,3); T5(2,3); T5(3,3)];
%Jacobcol7=[cross([T6(1,3); T6(2,3); T6(3,3)], [T7(1,4)-T6(1,4); T7(2,4)-T6(2,4); T7(3,4)-T6(3,4)]); T6(1,3); T6(2,3); T6(3,3)];
Jacob=[Jacobcol1 Jacobcol2 Jacobcol3 Jacobcol4 Jacobcol5 Jacobcol6];
%end

J = Jacobian(6, dH, T0, theta);

figure(1)
plot3([T0(1,4) T1(1,4) T2(1,4) T3(1,4) T4(1,4) T5(1,4) T6(1,4)], [T0(2,4) T1(2,4) T2(2,4) T3(2,4) T4(2,4) T5(2,4) T6(2,4)],[T0(3,4) T1(3,4) T2(3,4) T3(3,4) T4(3,4) T5(3,4) T6(3,4)], 'b')
 xlabel('X')
   ylabel('Y')
   xlim([-1 1])
   ylim([-1 1])
   zlim([-0.1 1.3])
   view(210,40)
grid on
hold on 
plot3([0],[0],[0], '-r*')


% plot3([0 u0(1)],[0 0],[0 0], 'r')
% plot3([0 0],[0 u0(2)],[0 0], 'r')
% plot3([0 0],[0 0],[0 u0(3)], 'r')

plot3([T6(1,4)], [T6(2,4)], [T6(3,4)], '-o')
plot3([T5(1,4)], [T5(2,4)], [T5(3,4)], '-o')
plot3([T4(1,4)], [T4(2,4)], [T4(3,4)], '-o')
plot3([T3(1,4)], [T3(2,4)], [T3(3,4)], '-o')
plot3([T2(1,4)], [T2(2,4)], [T2(3,4)], '-o')
plot3([T1(1,4)], [T1(2,4)], [T1(3,4)], '-o')