%Find the singularities of an elbow manipulator without the "wrist." Then,
    %Compute the isotropic points.
clear all;
%Segment Lengths
L0 = sym("L0"); L1 = sym("L1"); L2 = sym("L2");

%Points on the arm
Psh = [0 0 L0].'; Pe = [0 L1 L0].'; Pt = [0 L1+L2 L0].';

%omegas
w1 = [0 0 1].'; w2 = [1 0 0].'; w3 = [1 0 0].';

%joint angles
theta1 = sym("theta1"); theta2 = sym("theta2"); theta3 = sym("theta3");

%3-by-3 rotation matrices
ew1 = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1];
ew2 = [1 0 0; 0 cos(theta2) -sin(theta2); 0 sin(theta2) cos(theta2)];
ew3 = [1 0 0; 0 cos(theta3) -sin(theta3); 0 sin(theta3) cos(theta3)];

%4-by-4 exponential twists
gst_0 = [eye(3) Pt; 0 0 0 1];
ep1 = [ew1 Psh; 0 0 0 1];
ep2 = [ew2 Psh; 0 0 0 1];
ep3 = [ew3 Psh; 0 0 0 1];

%6-by-1 twist coordinates
Psi1 = [0 0 0 0 0 1].'; Psi2 = [0 0 0 1 0 0].'; Psi3 = Psi2;

%Finding "psi dagger" to calculate body jacobian
Psi1_dag = adjoint_inv(ep1*ep2*ep3*gst_0) * Psi1
Psi2_dag = adjoint_inv(ep2*ep3*gst_0) * Psi2
Psi3_dag = adjoint_inv(ep3*gst_0) * Psi3
Jb = [Psi1_dag Psi2_dag Psi3_dag];

sigma = svd(Jb)

