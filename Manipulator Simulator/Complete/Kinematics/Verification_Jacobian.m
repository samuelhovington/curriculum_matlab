close all; clc; clear all;
%File that runs the verification of the Jacobian function. Change indicated
%lines to make the script work correctly.

theta(:,1) = [180,270,90,270,270,270];
theta(:,2) = [180,180,180,180,180,180];
theta(:,3) = [350,100,45,100,170,0];
Rad2Deg = 180/pi;

% Dimensions of a Jaco2 Spherical wrist
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;
    

for ii = 1:3
% ---------------------------------------------------------------------------------------------------------------
% -------------------- Change the different parts of this section for your situation ----------------------------
% ---------------------------------------------------------------------------------------------------------------
% Choose your convention
%     Convention = 'Classic';
    Convention = 'Modified';
    
% Choose the unit of the angles
    AngleUnit = 'Degrees';
%     AngleUnit = 'Radians';

% Enter you number of DOF
    DOF = 6;
    
% Add the right value to pass from your physical angles to your algorithm
% angles
% Exemple : q(1) = Q1(index) + 360;
    q(1,ii) = theta(1,ii)+180;
    q(2,ii) = theta(2,ii)+90;
    q(3,ii) = theta(3,ii)+90;
<<<<<<< HEAD
    q(4,ii) = -theta(4,ii);
    q(5,ii) = -theta(5,ii);
    q(6,ii) = -(theta(6,ii)-90);
=======
    q(4,ii) = theta(4,ii);
    q(5,ii) = theta(5,ii)-180;
    q(6,ii) = theta(6,ii)+90;
<<<<<<< HEAD
>>>>>>> parent of 50c4af3... Adjustments
=======
>>>>>>> parent of 50c4af3... Adjustments
<<<<<<< HEAD
=======
    q(4,ii) = theta(4,ii);
    q(5,ii) = theta(5,ii)-180;
<<<<<<< HEAD
    q(6,ii) = -(theta(6,ii)+90);
>>>>>>> parent of e38b767... wrong
=======
    q(6,ii) = theta(6,ii)+90;
>>>>>>> parent of 50c4af3... Adjustments
=======
>>>>>>> parent of cf6f3e8... Revert "wrong"

% Define your DH parameters in the folowing matrix. You must keep the same
% syntax for the angles q and and use the dimensions of Jaco2 with a
% spherical wrist just above.
    %       alpha   a       d           theta 
    DH = [  0,      0,      -D1,        q(1,ii);
            pi/2,   0,      0,          q(2,ii);
            pi ,    D2,     -e2,        q(3,ii);
            3*pi/2,   0,      (D3+D4),   q(4,ii);
            pi/2,   0,      0,          q(5,ii);
            pi/2,   0,      (D5+D6),    q(6,ii)];
        
% Add your definition of the Trasformation Matrix between the world 
% arm's frame and the first DH frames that you just created 
    T0=[1   0   0   0;
        0   -1  0   0;
        0   0   -1  0;
        0   0   0   1];
    
% ----------------------------------------------------------------------------------------------------------------   
% ------------------------------ Make the change here to test your Jacobian --------------------------------------
% ---------------------------------------------------------------------------------------------------------------- 

% The fisrt line call a ready to work function that you can use to test your
% DH parameters.
% The second line call a function that you must complete to make it works.

% Comment/Uncomment the line that correspond to your situation
    J(:,:,ii) = Jacobian_complete(Convention, DOF, DH, T0, q(:,ii), AngleUnit);
%     J(:,:,ii) = Jacobian_to_complete(Convention, DOF, DH, T0, q(:,ii), AngleUnit);

% % ----------------------------------------------------------------------------------------------------------------   
% % ------------------------------- Do not change the code beyond this line --------------------------------------
% % ----------------------------------------------------------------------------------------------------------------
end

AnswerJ(:,:,1) = [

   
    0.2736   -0.3111    0.3111   -0.2638   -0.0000    0.0000;
   -0.4100   -0.0000    0.0000   -0.0000    0.0000   -0.0000;
         0   -0.4100    0.0000   -0.0000    0.2638   -0.0000;
         0   -0.0000    0.0000   -0.0000    1.0000    0.0000;
         0    1.0000   -1.0000    0.0000    0.0000   -1.0000;
   -1.0000   -0.0000    0.0000    1.0000    0.0000    0.0000]


AnswerJ(:,:,2) = [

    0.0098    0.9849   -0.5749    0.0000   -0.2638    0.0000;
    0.0000    0.0000   -0.0000   -0.0000   -0.0000    0.0000;
         0    0.0000    0.0000    0.0000    0.0000   -0.0000;
         0   -0.0000    0.0000    0.0000    0.0000   -0.0000;
         0    1.0000   -1.0000   -0.0000   -1.0000   -0.0000;
   -1.0000   -0.0000    0.0000   -1.0000    0.0000   -1.0000]


AnswerJ(:,:,3) = [

    -0.0660   -0.3862    0.3161   -0.0269   -0.0559    0.0000;
    0.0579   -0.0681    0.0557    0.0033    0.2499         0;
         0   -0.0684    0.4722   -0.0370    0.0632         0;
         0    0.1736   -0.1736    0.8067   -0.5864    0.7818;
         0   -0.9848    0.9848    0.1422    0.0729    0.3115;
   -1.0000   -0.0000   -0.0000   -0.5736   -0.8067   -0.5402]
