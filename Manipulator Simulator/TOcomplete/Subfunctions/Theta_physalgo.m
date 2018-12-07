%Function that receives a number in parameter (1, 2 or 3), a operation (1 
%for phys2algo or -1 for algo2phys) to do,  a convention and a set of 
%angles and that returns the desired angles

function q = Theta_physalgo(convention, angleUnit, num, op, theta)
    if strcmp(angleUnit, 'Degrees')
        if op == 1
            if strcmp(convention, 'Classic')
                q(1,1) = theta(1,1)+180;
                q(2,1) = theta(2,1)+90;
                q(3,1) = theta(3,1)+90;
                q(4,1) = theta(4,1);
                q(5,1) = theta(5,1);
                q(6,1) = (theta(6,1)-90);
            elseif strcmp(convention, 'Modified')
               if num == 2
                    q(1,1) = theta(1,1)+180;
                    q(2,1) = theta(2,1)+90;
                    q(3,1) = theta(3,1)+90;
                    q(4,1) = -theta(4,1);
                    q(5,1) = -theta(5,1);
                    q(6,1) = -(theta(6,1)-90);
               elseif num == 1
                    q(1,1) = theta(1,1)+180;
                    q(2,1) = theta(2,1)+90;
                    q(3,1) = theta(3,1)+90;
                    q(4,1) = theta(4,1);
                    q(5,1) = theta(5,1)-180;
                    q(6,1) = -(theta(6,1)-270);
               end
            else
            'Choose a convention between Classic and Modified'
            end
        elseif op == -1
             if strcmp(convention, 'Classic')
                q(1,1) = theta(1,1)-180;
                q(2,1) = theta(2,1)-90;
                q(3,1) = theta(3,1)-90;
                q(4,1) = theta(4,1);
                q(5,1) = theta(5,1);
                q(6,1) = (theta(6,1)+90);
            elseif strcmp(convention, 'Modified')
               if num == 2
                    q(1,1) = theta(1,1)-180;
                    q(2,1) = theta(2,1)-90;
                    q(3,1) = theta(3,1)-90;
                    q(4,1) = -theta(4,1);
                    q(5,1) = -theta(5,1);
                    q(6,1) = -(theta(6,1)-90);
               elseif num == 1
                    q(1,1) = theta(1,1)-180;
                    q(2,1) = theta(2,1)-90;
                    q(3,1) = theta(3,1)-90;
                    q(4,1) = theta(4,1);
                    q(5,1) = theta(5,1)+180;
                    q(6,1) = -(theta(6,1)-270);
               end
            else
            'Choose a convention between Classic and Modified'
            end
        end
    elseif strcmp(angleUnit, 'Radians')
                if op == 1
            if strcmp(convention, 'Classic')
                q(1,1) = theta(1,1)+pi;
                q(2,1) = theta(2,1)+pi/2;
                q(3,1) = theta(3,1)+pi;
                q(4,1) = theta(4,1);
                q(5,1) = theta(5,1);
                q(6,1) = (theta(6,1)-pi);
            elseif strcmp(convention, 'Modified')
               if num == 2
                    q(1,1) = theta(1,1)+pi;
                    q(2,1) = theta(2,1)+pi/2;
                    q(3,1) = theta(3,1)+pi/2;
                    q(4,1) = -theta(4,1);
                    q(5,1) = -theta(5,1);
                    q(6,1) = -(theta(6,1)-pi/2);
               elseif num == 1
                    q(1,1) = theta(1,1)+pi;
                    q(2,1) = theta(2,1)+pi/2;
                    q(3,1) = theta(3,1)+pi/2;
                    q(4,1) = theta(4,1);
                    q(5,1) = theta(5,1)-pi;
                    q(6,1) = -(theta(6,1)-3*pi/2);
               end
            else
            'Choose a convention between Classic and Modified'
            end
        elseif op == -1
             if strcmp(convention, 'Classic')
                q(1,1) = theta(1,1)-pi;
                q(2,1) = theta(2,1)-pi/2;
                q(3,1) = theta(3,1)-pi/2;
                q(4,1) = theta(4,1);
                q(5,1) = theta(5,1);
                q(6,1) = (theta(6,1)+pi/2);
            elseif strcmp(convention, 'Modified')
               if num == 2
                    q(1,1) = theta(1,1)-pi;
                    q(2,1) = theta(2,1)-pi/2;
                    q(3,1) = theta(3,1)-pi/2;
                    q(4,1) = -theta(4,1);
                    q(5,1) = -theta(5,1);
                    q(6,1) = -(theta(6,1)-pi/2);
               elseif num == 1
                    q(1,1) = theta(1,1)-pi;
                    q(2,1) = theta(2,1)-pi/2;
                    q(3,1) = theta(3,1)-pi/2;
                    q(4,1) = theta(4,1);
                    q(5,1) = theta(5,1)+pi;
                    q(6,1) = -(theta(6,1)-3*pi/2);
               end
            else
            'Choose a convention between Classic and Modified'
            end
                end
    else
        'Choose a unit between Radians and Degrees'
    end
end