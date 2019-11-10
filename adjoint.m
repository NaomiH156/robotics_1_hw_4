function q = adjoint(g)
q = eye(6);
R = g(1:3,1:3);
p_hat = skew_matrix(g(1:3,4));

q(1:3,1:3) = R;
q(1:3,4:6) = p_hat*R;
q(4:6,4:6) = R;




end

