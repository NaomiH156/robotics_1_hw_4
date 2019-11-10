function R = Rz(theta)
    R = zeros(3,3);
    R(1:2, 1:2) = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    R(3,3) = 1;
end