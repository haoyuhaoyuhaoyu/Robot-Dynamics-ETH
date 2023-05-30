function J_P = jointToPosJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector translation which maps joint
  % velocities to end-effector linear velocities in I frame.

  % Compute the translational jacobian.
  J_P = zeros(3, 6);
  % default in frame I
  % get transform
  T_I0 = getTransformI0();
  T_01 = jointToTransform01(q);
  T_12 = jointToTransform12(q);
  T_23 = jointToTransform23(q);
  T_34 = jointToTransform34(q);
  T_45 = jointToTransform45(q);
  T_56 = jointToTransform56(q);
  T_6E = getTransform6E();
  % get transform to frame I
  T_I1 = T_I0*T_01;
  T_I2 = T_I1*T_12;
  T_I3 = T_I2*T_23;
  T_I4 = T_I3*T_34;
  T_I5 = T_I4*T_45;
  T_I6 = T_I5*T_56;
  T_IE = T_I6*T_6E;
  % get each pos in frame I
  I_rI1 = T_I1(1:3, 4);
  I_rI2 = T_I2(1:3, 4);
  I_rI3 = T_I3(1:3, 4);
  I_rI4 = T_I4(1:3, 4);
  I_rI5 = T_I5(1:3, 4);
  I_rI6 = T_I6(1:3, 4);
  I_rIE = T_IE(1:3, 4);
  % get rotmat C
  C_I0 = T_I0(1:3, 1:3);
  C_01 = T_01(1:3, 1:3);
  C_12 = T_12(1:3, 1:3);
  C_23 = T_23(1:3, 1:3);
  C_34 = T_34(1:3, 1:3);
  C_45 = T_45(1:3, 1:3);
  C_56 = T_56(1:3, 1:3);
  C_6E = T_6E(1:3, 1:3);
  % get unit w in each body frame
  n1 = [0,0,1]';
  n2 = [0,1,0]';
  n3 = [0,1,0]';
  n4 = [1,0,0]';
  n5 = [0,1,0]';
  n6 = [1,0,0]';
  % get rotmat trans to frame I
  C_I1 = C_I0*C_01;
  C_I2 = C_I1*C_12;
  C_I3 = C_I2*C_23;
  C_I4 = C_I3*C_34;
  C_I5 = C_I4*C_45;
  C_I6 = C_I5*C_56;
  C_IE = C_I6*C_6E;
  % transform n in each body frame to frame I
  I_n1 = C_I1*n1;
  I_n2 = C_I2*n2;
  I_n3 = C_I3*n3;
  I_n4 = C_I4*n4;
  I_n5 = C_I5*n5;
  I_n6 = C_I6*n6;
  % compute jacobian matrix
  J_P = [cross(I_n1, I_rIE-I_rI1),...
         cross(I_n2, I_rIE-I_rI2),...
         cross(I_n3, I_rIE-I_rI3),...
         cross(I_n4, I_rIE-I_rI4),...
         cross(I_n5, I_rIE-I_rI5),...
         cross(I_n6, I_rIE-I_rI6),...
         ];
end