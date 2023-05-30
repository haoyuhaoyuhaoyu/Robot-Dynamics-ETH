function T45 = jointToTransform45(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 4. T_45
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  if (length(q)>1)
      q = q(5);
  end
  T45 = zeros(4);
  C45 = [ cos(q),   0,   sin(q);
               0,   1,        0;
         -sin(q),   0,   cos(q)];  % rot y 
  r45 = [0.168; 0; 0];
  T45(1:3, 1:3) = C45;
  T45(1:3, 4) = r45;
  T45(4, 4) = 1;
end

