%A script to derive the spatial jacobian of the inverse elbow manipulator

%Segment lengths
L0 = sym("L0"); L1 = sym("L1"); L2 = sym("L2"); L3 = sym("L3");

%Joint angles
theta1 = sym("theta1");
theta2 = sym("theta2");
theta3 = sym("theta3");
theta4 = sym("theta4");
theta5 = sym("theta5");
theta6 = sym("theta6");

%axes of rotation
w1 = [0 0 1].';
w2 = [0 1 0].';
w3 = [1 0 0].';
w4 = [1 0 0].';
w5 = [1 0 0].';
w6 = [0 1 0].';

%3-by-3 revolute transformation matrices
% ew1 = Rz(theta1); 
% ew2 = Ry(theta2);
% ew3 = Rx(theta3);
% ew4 = Rx(theta4);
% ew5 = Rx(theta5);
% ew6 = Ry(theta6);
%Rx, Ry, Rz aren't playing nice with arguments of type sym so im just gonna
    %hard code them:
ew1 = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1];
ew2 = [cos(theta2) 0 sin(theta2); 0 1 0; -sin(theta2) 0 cos(theta2)];
ew3 = [1 0 0; 0 cos(theta3) -sin(theta3); 0 sin(theta3) cos(theta3)];
ew4 = [1 0 0; 0 cos(theta4) -sin(theta4); 0 sin(theta4) cos(theta4)];
ew5 = [1 0 0; 0 cos(theta5) -sin(theta5); 0 sin(theta5) cos(theta5)];
ew6 = [cos(theta6) 0 sin(theta6); 0 1 0; -sin(theta6) 0 cos(theta6)];

%Some points along the manipulator:
Psh = [0 0 L0].'; %"shoulder position"
Pe = [0 L1 L0].'; %"elbow position"
Pw = [0 L1+L2 L0].'; %"wrist position"
Pt = [0 L1+L2+L3 L0].'; %"Tool position"
q1 = Psh; q2 = Psh; q3 = Psh; q4 = Pe; q5 = Pw; q6 = Pt;

%Zero angle configuration:
gst_0 = [eye(3) Pt; 0 0 0 1];

%4-by-4 exponential twist matrices
ep1 = [ew1 q1; 0 0 0 1]; 
ep2 = [ew2 q2; 0 0 0 1];
ep3 = [ew3 q3; 0 0 0 1];
ep4 = [ew4 q4; 0 0 0 1];
ep5 = [ew5 q5; 0 0 0 1];
ep6 = [ew6 q6; 0 0 0 1];

%"q-prime"
q2p = q1 + ew1*q2
q3p = q2p + ew1*ew2*q3
q4p = q3p + ew1*ew2*ew3*q4
q5p = q4p + ew1*ew2*ew3*ew4*q5
q6p = q5p + ew1*ew2*ew3*ew4*ew5*q6
%I think technically that the above should be ep instead of ew, but since
    %all of the joints are revolute we're allowed to cheat and get away
    %with it. If we try and use ep, we run into issues with q being 3-by-1
    %when the ep matrices are 4-by-4.

%"omega prime"
w2p = ew1*w2
w3p = ew1*ew2*w3
w4p = ew1*ew2*ew3*ew4*w4
w5p = ew1*ew2*ew3*ew4*ew5*w5
w6p = ew1*ew2*ew3*ew4*ew5*ew6*w6

%transformed 6-by-1 twist coordinates
Psi1 = [0 0 0 0 0 1].';
Psi2p = [cross(w2p, q2p); w2p];
Psi3p = [cross(w3p, q3p); w3p];
Psi4p = [cross(w4p, q4p); w4p];
Psi5p = [cross(w5p, q5p); w5p];
Psi6p = [cross(w6p, q6p); w6p];

Js = [Psi1 Psi2p Psi3p Psi4p Psi5p Psi6p]