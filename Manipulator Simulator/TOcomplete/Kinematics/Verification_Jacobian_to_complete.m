close all; clc; clear all;
%File that runs the verification of Jacobian function. Change indicated 
%lines to make the script work
%correctly.

%The answers of the verification depends on these angles so do not refer to
%AnswerJ if you change these angles
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
    Convention = 'Classic';
%     Convention = 'Modified';
    
% Choose the unit of the angles
    AngleUnit = 'Degrees';
%     AngleUnit = 'Radians';

% Enter you number of DOF
    DOF = 6;
    
% Add the right value to pass from your physical angles to your algorithm
% angles
% Exemple : q(1) = Q1(index) + 360;
    q(1,ii) = theta(1,ii);
    q(2,ii) = theta(2,ii);
    q(3,ii) = theta(3,ii);
    q(4,ii) = theta(4,ii);
    q(5,ii) = theta(5,ii);
    q(6,ii) = theta(6,ii);

% Define your DH parameters in the folowing matrix. You must keep the same
% syntax for the angles q and and use the dimensions of Jaco2 with a
% spherical wrist just above.
    %       alpha   a       d           theta 
    DH = [  0,      0,      0,          q(1,ii);
            0,      0,      0,          q(2,ii);
            0,      0,      0,          q(3,ii);
            0,      0,      0,          q(4,ii);
            0,      0,      0,          q(5,ii);
            0,      0,      0,          q(6,ii)];
        
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

txt2 = 'Your matrix J(:,:,i) should be the same as the matrix AnswerJ(:,:,i).';
txt1 = 'The goal of this exercise is to verify your Jacobian function by verifying that it made you obtain the correct jacobian matrix.';
txt3 = '';
txt4 = 'When you are done reading, press ok and press any button in the command window to start the program.';
uiwait(msgbox({txt1 txt2 txt3 txt4}, 'Jacobian'));

AnswerJ(:,:,1) = [

    0.1722    0.1099    0.0017   -0.0424   -0.0978         0;
   -0.0697    0.3714    0.0056    0.0177   -0.1694    0.0000;
         0    0.1849   -0.0686    0.0066   -0.1770    0.0000;
         0    0.9589   -0.9589   -0.2273    0.9141    0.1646;
         0   -0.2837    0.2837   -0.7682   -0.3803    0.6656;
   -1.0000   -0.0000    0.0000    0.5985   -0.1410   -0.7280]


AnswerJ(:,:,2) = [

   -0.3165   -0.1535    0.1330    0.0733    0.1481    0.0000;
   -0.0931   -0.5189    0.4497   -0.1982    0.0547    0.0000;
         0   -0.2771   -0.1265   -0.0000   -0.2113   -0.0000;
         0    0.9589   -0.9589   -0.0000   -0.3466   -0.7515;
         0   -0.2837    0.2837    0.0000    0.9380   -0.2777;
   -1.0000   -0.0000    0.0000   -1.0000    0.0000   -0.5985]


AnswerJ(:,:,3) = [

    0.0525    0.2912   -0.0463    0.0638   -0.0428   -0.0000;
    0.0064    0.3898   -0.0620   -0.0464    0.1520    0.0000;
         0    0.0382   -0.0654   -0.0463   -0.2113    0.0000;
         0    0.8011   -0.8011   -0.5984    0.6975    0.6979;
         0   -0.5985    0.5985   -0.8009   -0.5071    0.6409;
   -1.0000   -0.0000    0.0000   -0.0221   -0.5062    0.3196]
