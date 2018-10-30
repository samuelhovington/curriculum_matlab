function theta = ThetaAlgo(theta)
%     q(1) = theta(1)+180;
%     q(2) = theta(2)+90;
%     q(3) = theta(3)+90;
%     q(4) = theta(4);
%     q(5) = theta(5)+180;
%     q(6) = -(theta(6)+90);
    Deg2Rad = pi/180;
    theta(1) = Deg2Rad * (theta(1) + 180);
theta(2) = Deg2Rad * (theta(2) +90);
theta(3) = Deg2Rad * (theta(3) + 90);
theta(4) = Deg2Rad * theta(4);
theta(5) = Deg2Rad * theta(5);
theta(6) = Deg2Rad * (theta(6) - 90);
end