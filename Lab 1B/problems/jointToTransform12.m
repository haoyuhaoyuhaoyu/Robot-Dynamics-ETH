function T12 = jointToTransform12(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 1. T_12
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  if (length(q)>1)
      q = q(2);
  end
  T12 = zeros(4);
  C12 = [ cos(q),   0,   sin(q);
               0,   1,        0;
         -sin(q),   0,   cos(q)];  % rot y 
  r12 = [0;0;0.145];
  T12(1:3, 1:3) = C12;
  T12(1:3, 4) = r12;
  T12(4, 4) = 1;
end