%Function that receives a number in parameter (1, 2 or 3) and a convention
%and that returns the corresponding set of DH parameters

function dh = DH(convention, num)
D1 = 0.2755;
D2 = 0.4100;
D3 = 0.2073;
D4 = 0.1038;
D5 = 0.1038;
D6 = 0.16;
e2 = 0.0098;
    if strcmp(convention, 'Classic')
        dh = [  pi/2,   0,      -D1;
                pi,     D2,      0;
                pi/2,   0,      -e2;
                pi/2,   0,      -(D3+D4);
                pi/2,   0,      0;
                pi,     0,      -(D5+D6)];
    elseif strcmp(convention, 'Modified')
       if num == 2;
        dh = [  0,      0,      -D1;
                pi/2,   0,      0;
                pi,     D2,      -e2;
                3*pi/2, 0,      (D3+D4);
                pi/2,   0,      0;
                pi/2,   0,      (D5+D6)];
       elseif num == 1;
        dh = [  0,      0,      -D1;
                pi/2,   0,      0;
                pi,     D2,     -e2;
                pi/2,   0,      -(D3+D4);
                pi/2,   0,      0;
                pi/2,   0,      (D5+D6)];
       end
    else
    'Choose a convention between Classic and Modified'
    end
end