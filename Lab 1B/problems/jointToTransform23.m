function T23 = jointToTransform23(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 3 to frame 2. T_23
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  if (length(q)>1)
      q = q(3);
  end
  T23 = zeros(4);
  C23 = [ cos(q),   0,   sin(q);
               0,   1,        0;
         -sin(q),   0,   cos(q)];  % rot y 
  r23 = [0; 0; 0.27];
  T23(1:3, 1:3) = C23;
  T23(1:3, 4) = r23;
  T23(4, 4) = 1;
end
