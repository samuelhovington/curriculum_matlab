function T = Transformi_i1(convention, angleUnit, DH, q)
    for i=1:6
       alpha(i) = DH(i,1);
       d(i) = DH(i,3);
       a(i) = DH(i,2);
    end    

    if strcmp(angleUnit, 'Degrees')
        q = q*pi/180;
        angleUnit = 'Radians';
    end
    if strcmp(convention,'Modified')
        %Transformation matrix ^{n-1}T^{n}
        for i=1:6
            T(:,:,i)=[cos(q(i)) -sin(q(i)) 0 a(i);...
                sin(q(i))*cos(alpha(i)) cos(alpha(i))*cos(q(i)) -sin(alpha(i)) -d(i)*sin(alpha(i)); ...
                sin(q(i))*sin(alpha(i)) cos(q(i))*sin(alpha(i)) cos(alpha(i)) d(i)*cos(alpha(i));...
                0 0 0 1];
        end

    elseif strcmp(convention, 'Classic')
        
        %Transformation matrix ^{n-1}T^{n}
        for i=1:6
            T(:,:,i)=[cos(q(i)) -sin(q(i))*cos(alpha(i)) sin(q(i))*sin(alpha(i)) a(i)*cos(q(i));...
                sin(q(i)) cos(alpha(i))*cos(q(i)) -cos(q(i))*sin(alpha(i)) a(i)*sin(q(i)); ...
                0 sin(alpha(i)) cos(alpha(i)) d(i);...
                0 0 0 1];
            
        end
    end
end