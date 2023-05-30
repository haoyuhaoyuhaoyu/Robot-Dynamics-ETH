function T56 = jointToTransform56(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 6 to frame 5. T_56
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  if (length(q)>1)
      q = q(6);
  end
  T56 = zeros(4);
  C56 = [      1,        0,        0;
               0,   cos(q),  -sin(q);
               0,   sin(q),   cos(q)];  % rot x 
  r56 = [0.072; 0; 0];
  T56(1:3, 1:3) = C56;
  T56(1:3, 4) = r56;
  T56(4, 4) = 1;
end
