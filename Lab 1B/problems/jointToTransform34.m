function T34 = jointToTransform34(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 4 to frame 3. T_34
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  if (length(q)>1)
      q = q(4);
  end
  T34 = zeros(4);
  C34 = [      1,        0,        0;
               0,   cos(q),  -sin(q);
               0,   sin(q),   cos(q)];  % rot x 
  r34 = [0.134; 0; 0.07];
  T34(1:3, 1:3) = C34;
  T34(1:3, 4) = r34;
  T34(4, 4) = 1;
end

