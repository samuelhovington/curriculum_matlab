clear all;close all;

theta = [180;270;180;270;270;270];
dtheta = [0;0;0;0;0;0];
ddtheta = [0;0;0;0;0;0];
theta = theta*pi/180;


m = [0.18198+0.570,0.42399+0.570,0.21100+0.570,0.09184+0.357, 0.09184+0.357,0.727+0.357];
D(1) = 0.2755;
D(2) = 0.4100;
D(3) = 0.2073;
D(4) = 0.1038;
D(5) = 0.1038;
D(6) = 0.16;
e2 = 0.0098;
g = 9.81;
Ipm = [0.0001020703 0.0003248936 0.0003536944 -5.214394e-9 -0.000021186541 -4.141348e-9];
Ic = [Ipm(2), Ipm(3), Ipm(1), -Ipm(6), Ipm(4), -Ipm(5)];
I(:,:,1) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ipm = [0.0037862305 0.0037251264 0.000069776668 0 0 5.903745e-8];
Ic = [Ipm(3), Ipm(2), Ipm(1), Ipm(6), Ipm(5), Ipm(4)];
I(:,:,2) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ipm = [0.0005109958 0.0004804467 0.00004810136 1.922692e-9 -0.000001586224 1.960234e-9];
Ic = [Ipm(2), Ipm(3), Ipm(1), -Ipm(6), -Ipm(4), Ipm(5)];
I(:,:,3) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ipm = [0.0001933181 0.00007567544 0.0001953757 -0.00003628615 -1.204e-8 1.37e-8];
Ic = [Ipm(3), Ipm(1), Ipm(2), -Ipm(5), Ipm(6), -Ipm(4)];
I(:,:,4) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ic = [Ipm(3), Ipm(2), Ipm(1), -Ipm(6), -Ipm(5), Ipm(4)];
I(:,:,5) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];
Ipm = [0.00004659624 0.00002647306 0.00005247987 0.00000413032 -1.0573e-7 -3.244e-8];
Ic = [Ipm(3), Ipm(1), Ipm(2), Ipm(5), -Ipm(6), -Ipm(4)];
I(:,:,6) = [Ic(1), Ic(4), Ic(5); Ic(4), Ic(2), Ic(6); Ic(5), Ic(6), Ic(3)];

rcmi_i(:,:,1) = [0;0;D(1)/2] + [0.23712255; -10.29160997; 69.17216347].*10^-3;
rcmi_i(:,:,2) = [205.23664355, -0.02629337, 22.30776575].*10^-3;
rcmi_i(:,:,3) = [0.07073408, 64.37250505, -19.56874086].*10^-3;
rcmi_i(:,:,4) = [0.01, -13.28, -62.52].*10^-3;
rcmi_i(:,:,5) = [0.01, D(5) - 62.52, -13.28].*10^-3;
rcmi_i(:,:,6) = [-6.84, -0.03, 82.22].*10^-3;

r_i_im1(:,:,1) = [0;0;0];
r_i_im1(:,:,2) = [0;0;-D(1)];
r_i_im1(:,:,3) = [D(2);0;0];
r_i_im1(:,:,4) = [0;D(3);-e2];
r_i_im1(:,:,5) = [0;0;-D(4)];
r_i_im1(:,:,6) = [0;-D(5);0];
r_i_im1(:,:,7) = [0;0;D(6)];

w0 = [0;0;0];
dw0 = [0;0;0];
dv0 = [0;0;-g];
fl = [0;0;0];
nl = [0;0;0];
% ---------------------------------------------------------------------------------------------------------------
% -------------------- Change the different parts of this section for your situation ----------------------------
% ---------------------------------------------------------------------------------------------------------------
% Choose your convention
%     Convention = 'Classic';
    Convention = 'Modified';
    
% Choose the unit of the angles
%     AngleUnit = 'Degrees';
    AngleUnit = 'Radians';

% Enter you number of DOF
    DOF = 6;
    
% Add the right value to pass from your physical angles to your algorithm
% angles
% Exemple : q(1) = Q1(index) + 360;
    theta(:,1) = Theta_physalgo(Convention, AngleUnit,1,1,theta);
% Define your DH parameters in the folowing matrix. You must keep the same
% syntax for the angles q and and use the dimensions of Jaco2 with a
% spherical wrist just above.
%       alpha   a       d 
DH = DH(Convention, 1);
% Add your definition of the Trasformation Matrix between the world 
% arm's frame and the first DH frames that you just created 
    TW0=[1   0   0   0;
        0   -1  0   0;
        0   0   -1  0;
        0   0   0   1];

T = FKforDynmcs(Convention, DOF, DH, TW0, theta, AngleUnit);
for index = 1:DOF
    R_im1_i(:,:,index) = T(1:3,1:3,index);
end

w_i_i(:,:,1) = transpose(R_im1_i(:,:,1)) * w0 + [0;0;dtheta(1)];
dw_i_i(:,:,1) = transpose(R_im1_i(:,:,1))*dw0 + cross(transpose(R_im1_i(:,:,1))*w0,[0;0;dtheta(1)]) + [0;0;ddtheta(1)]; 
dv_i_i(:,:,1) = transpose(R_im1_i(:,:,1))*(cross(dw0, r_i_im1(:,:,1))+cross(w0,cross(w0,r_i_im1(:,:,1))) + dv0);
dv_cmi_i(:,:,1) = cross(dw_i_i(:,:,1), rcmi_i(:,:,1)) + cross(w_i_i(:,:,1), cross(w_i_i(:,:,1),rcmi_i(:,:,1))) + dv_i_i(:,:,1);
F_i(:,:,1) = m(1)*dv_cmi_i(:,:,1);
M_i(:,:,1) = I(:,:,1)*dw_i_i(:,:,1) + cross(w_i_i(:,:,1), I(:,:,1)*w_i_i(:,:,1));

for index = 2:6
    w_i_i(:,:,index) = transpose(R_im1_i(:,:,index)) * w_i_i(:,:,index-1) + [0;0;dtheta(index)];
    dw_i_i(:,:,index) = transpose(R_im1_i(:,:,index))*dw_i_i(:,:,index-1) + cross(transpose(R_im1_i(:,:,index))*w_i_i(:,:,index-1),[0;0;dtheta(index)]) + [0;0;ddtheta(index)];
    dv_i_i(:,:,index) = transpose(R_im1_i(:,:,index))*(cross(dw_i_i(:,:,index-1), r_i_im1(:,:,index))+cross(w_i_i(:,:,index-1),cross(w_i_i(:,:,index-1), r_i_im1(:,:,index))) + dv_i_i(:,:,index-1));
    dv_cmi_i(:,:,index) = cross(dw_i_i(:,:,index), rcmi_i(:,:,index)) + cross(w_i_i(:,:,index), cross(w_i_i(:,:,index),rcmi_i(:,:,index))) + dv_i_i(:,:,index);
    F_i(:,:,index) = m(index)*dv_cmi_i(:,:,index);
    M_i(:,:,index) = I(:,:,index)*dw_i_i(:,:,index) + cross(w_i_i(:,:,index), I(:,:,index)*w_i_i(:,:,index));
end

f_i(:,:,6) = fl + F_i(:,:,6);
n_i(:,:,6) = M_i(:,:,6) + nl + cross(rcmi_i(:,:,6), F_i(:,:,6)) + fl;
tau_v(6,1) = n_i(3,:,6);
for index =5:-1:1
    f_i(:,:,index) = R_im1_i(:,:,index+1)*f_i(:,:,index+1)  + F_i(:,:,index);
    n_i(:,:,index) = M_i(:,:,index) + R_im1_i(:,:,index+1)*n_i(:,:,index+1) + cross(rcmi_i(:,:,index), F_i(:,:,index)) + cross(r_i_im1(:,:,index+1), R_im1_i(:,:,index+1)*f_i(:,:,index+1));
    tau_v(index, 1) = transpose(n_i(3,:,index));
end
tau_v

%% Verification parameters
T0(:,:,1) = TW0*T(:,:,1);
rcmi_0(:,:,1) = rcmi_i(:,:,1);
    for i = 2:6
        T0(:,:,i) = T0(:,:,i-1)*T(:,:,i);
%          R0(:,:,i) = [T0(1,1,i), T0(1,2,i), T0(1,3,i);T0(2,1,i),T0(2,2,i),T0(2,3,i);T0(3,1,i),T0(3,2,i),T0(3,3,i)];
        R0(:,:,i) = [T0(1:3,1,i), T0(1:3,2,i), T0(1:3,3,i)];

        ri_0(:,:,i) = [T0(1,4,i), T0(2,4,i), T0(3,4,i)];
    end
   
    J0 = r_i_im1(:,:,1);
    J(:,:,1) = J0 - r_i_im1(:,:,2);
    J(:,:,2) =  J(:,:,1) + R0(:,:,2)*r_i_im1(:,:,3);
    J(:,:,3) =  J(:,:,2) + R0(:,:,3)*r_i_im1(:,:,4);
    J(:,:,4) =  J(:,:,3) + R0(:,:,4)*r_i_im1(:,:,5);
    J(:,:,5) =  J(:,:,4) + R0(:,:,5)*r_i_im1(:,:,6);
    J(:,:,6) =  J(:,:,5) + R0(:,:,6)*r_i_im1(:,:,7);
    
    for i = 2:6
       rcmi_0(:,:,i) =  (R0(:,:,i) * rcmi_i(:,:,i)) + J(:,:,i-1);  
    end
%% GRAPHING THE ROBOT


    %Physical positions of the origins of the frames to view the robot
    J6 = [T0(1:3,4,6)];
    J5 = [T0(1:3,4,5)];
    J4 = [T0(1:3,4,4)];
    J3 = [T0(1:3,4,3)];
    J2 = [T0(1:3,4,2)];
    J1 = [T0(1:3,4,1)];
    J0 = [TW0(1:3,4)];
    J0 = r_i_im1(:,:,1);
    J1 = J0 - r_i_im1(:,:,2);
    J2 = J1 +  R0(:,:,2)*r_i_im1(:,:,3);
    J3 =  J2 + R0(:,:,3)*r_i_im1(:,:,4);
    J4 =  J3 + R0(:,:,4)*r_i_im1(:,:,5);
    J5 =  J4 + R0(:,:,5)*r_i_im1(:,:,6);
    J6 =  J5 + R0(:,:,6)*r_i_im1(:,:,7);
   coordinates = [J0,J(:,:,1),J(:,:,2),J(:,:,3),J(:,:,4),J(:,:,5),J(:,:,6)];
        
 
   EndEffector(index,:)=coordinates(:,5);
   
   figure(1);

   plot3(coordinates(1,:),coordinates(2,:),coordinates(3,:))
   hold on
   view(150,40) 
   set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
%    set ( gca, 'xdir', 'reverse' )
%     set ( gca, 'ydir', 'reverse' )
   plot3(coordinates(1,1),coordinates(2,1),coordinates(3,1), '-r*')
   axis equal
   set(findall(gca, 'Type', 'Line'),'LineWidth',2);
   xlabel('X')
   ylabel('Y')
   xlim([-1 1])
   ylim([-1 1])
   zlim([-1.3 1.3])
   plot3(EndEffector(:,1),EndEffector(:,2),EndEffector(:,3))
   %plot3(Pee.ans(2,:),Pee.ans(3,:),Pee.ans(4,:))
   grid on
   
   for index = 1:6
       plot3(J(1,1,index), J(2,1,index), J(3,1,index), '-x')
      plot3(rcmi_0(1,1,index), rcmi_0(2,1,index), rcmi_0(3,1,index), '-o') 
   end
   
   hold off