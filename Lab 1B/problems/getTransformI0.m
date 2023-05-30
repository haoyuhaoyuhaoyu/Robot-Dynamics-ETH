function TI0 = getTransformI0()
  % Input: void
  % Output: homogeneous transformation Matrix from frame 0 to the inertial frame I. T_I0
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  TI0 = zeros(4);
  CI0 = eye(3);
  rI0 = [0;0;0];
  TI0(1:3, 1:3) = CI0;
  TI0(1:3, 4) = rI0;
  TI0(4, 4) = 1;
end
