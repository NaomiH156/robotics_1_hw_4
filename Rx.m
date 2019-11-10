function R = Rx(theta)
    R = eye(3);
    R(2:3, 2:3) = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end