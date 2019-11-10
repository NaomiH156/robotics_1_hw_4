%A script to find the singularities of the XYZ Euler angles

alpha = sym("alpha"); beta = sym("beta"); gamma = sym("gamma");

Rx_alpha = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
Ry_beta = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
Rz_gamma = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];

R_total = Rx_alpha*Ry_beta*Rz_gamma

%sigmas = svd(R_total)

w1 = [1 0 0].'; w2 = [0 1 0].'; w3 = [0 0 1].';
Psi1 = [0; 0; 0; w1]; Psi2 = [0; 0; 0; w2]; Psi3 = [0; 0; 0; w3];
J = [Psi1 Psi2 Psi3];

%4-by-4 exponential twist matrices
q = [0 0 0].';
ep1 = [Rx_alpha q; 0 0 0 1];
ep2 = [Ry_beta q; 0 0 0 1];
ep3 = [Rz_gamma q; 0 0 0 1];
gst_0  = eye(4);

%find body jacobian
Psi1_dag = adjoint_inv(ep1*ep2*ep3*gst_0) * Psi1
Psi2_dag = adjoint_inv(ep2*ep3*gst_0) * Psi2
Psi3_dag = adjoint_inv(ep3*gst_0) * Psi3

Jb = [Psi1_dag Psi2_dag Psi3_dag];

sigmas = svd(Jb)