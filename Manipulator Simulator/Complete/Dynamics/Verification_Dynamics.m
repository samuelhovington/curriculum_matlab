%File that executes the dynamics of a 6 DOF S Ultra Lightweight Robotic Arm
%using the Newton-Euler iterative algorithm.

%Created on 2018-11-28 by Simon Michaud @Kinova
%Modified on 2018-11-28
clear all;close all;
txt2 = 'If everything is correct, tau_v should be close (+- 1.5 Nm) to the right answers displayed by the messages.';
txt1 = 'The goal of this exercise is to verify your dynamics algroithm by using test conditions to verify the torques.';
txt3 = '';
txt4 = 'When you are done reading, press ok to start the program.';
uiwait(msgbox({txt1 txt2 txt3 txt4}, 'Verification Dynamics'));
% ----------------------------------------------------------------------------------------------------------------
% ------------------------------------- Statics Verification Point 1 ---------------------------------------------
% ----------------------------------------------------------------------------------------------------------------

theta = [180;270;180;270;270;270];
dtheta = [0;0;0;0;0;0];
ddtheta = [0;0;0;0;0;0];
theta = theta*pi/180;

AngleUnit = 'Radians';

tau_calcul(:,1) = DynamicsJaco6DOFS_complete(theta, dtheta, ddtheta, AngleUnit)
message1 = 'The right answer should be around [0.2;-18.7;6;-1.7;-0.1;0.1]'

% ----------------------------------------------------------------------------------------------------------------
% ------------------------------------- Statics Verification Point 2 ---------------------------------------------
% ----------------------------------------------------------------------------------------------------------------

theta = [180;180;180;270;270;270];
dtheta = [0;0;0;0;0;0];
ddtheta = [0;0;0;0;0;0];
theta = theta*pi/180;

AngleUnit = 'Radians';

tau_calcul(:,2) = DynamicsJaco6DOFS_complete(theta, dtheta, ddtheta, AngleUnit)
message2 = 'The right answer should be around [0;0;0;-0.1;-2;0.1]'

% ----------------------------------------------------------------------------------------------------------------
% ------------------------------------- Statics Verification Point 3 ---------------------------------------------
% ----------------------------------------------------------------------------------------------------------------

theta = [283;162;43;265;257;288];
dtheta = [0;0;0;0;0;0];
ddtheta = [0;0;0;0;0;0];
theta = theta*pi/180;

AngleUnit = 'Radians';

tau_calcul(:,3) = DynamicsJaco6DOFS_complete(theta, dtheta, ddtheta, AngleUnit)
message3 = 'The right answer should be around [0.8;-1.4;5.3;-1.5;0.6;0.1]'

% ----------------------------------------------------------------------------------------------------------------
% ------------------------------------- Statics Verification Point 4 ---------------------------------------------
% ----------------------------------------------------------------------------------------------------------------

theta = [178;180;94;180;181;288];
dtheta = [0;0;0;0;0;0];
ddtheta = [0;0;0;0;0;0];
theta = theta*pi/180;

AngleUnit = 'Radians';

tau_calcul(:,4) = DynamicsJaco6DOFS_complete(theta, dtheta, ddtheta, AngleUnit)
message4 = 'The right answer should be around [0;-8;7;0;1.6;0.1]'

% ----------------------------------------------------------------------------------------------------------------
% ------------------------------------- Statics Verification Point 5 ---------------------------------------------
% ----------------------------------------------------------------------------------------------------------------

theta = [337;221;53;252;264;337];
dtheta = [0;0;0;0;0;0];
ddtheta = [0;0;0;0;0;0];
theta = theta*pi/180;

AngleUnit = 'Radians';

tau_calcul(:,5) = DynamicsJaco6DOFS_complete(theta, dtheta, ddtheta, AngleUnit)
message5 = 'The right answer should be around [0;-10;0.1;-0.4;1.5;0.1]'

