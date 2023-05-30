function T6E = getTransform6E()
  % Input: void
  % Output: homogeneous transformation Matrix from the end-effector frame E to frame 6. T_6E
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T6E = zeros(4);
  C6E = eye(3);
  r6E = [0;0;0];
  T6E(1:3, 1:3) = C6E;
  T6E(1:3, 4) = r6E;
  T6E(4, 4) = 1;
end

