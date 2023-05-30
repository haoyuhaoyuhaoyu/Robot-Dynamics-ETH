function T01 = jointToTransform01(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 1 to frame 0. T_01
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  if (length(q)>1)
      q = q(1);
  end
  T01 = zeros(4);
%  C01 = [ cos(q),   0,   sin(q);
%               0,   1,        0;
%         -sin(q),   0,   cos(q)];  % rot y 
  C01 = [cos(q), -sin(q), 0;
         sin(q),  cos(q), 0;
              0,       0, 1];  % rot z
  r01 = [0;0;0.145];
  T01(1:3, 1:3) = C01;
  T01(1:3, 4) = r01;
  T01(4, 4) = 1;
end