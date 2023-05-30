function quat = jointToQuat(q)
  % Input: joint angles
  % Output: quaternion representing the orientation of the end-effector
  % q_IE.
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  quat = zeros(4,1);
  C_IE = jointToRotMat(q);
  quat(1) = 0.5*sqrt(C_IE(1,1)+C_IE(2,2)+C_IE(3,3)+1);
  quat(2) = 0.5*(sign(C_IE(3,2)-C_IE(2,3))*sqrt(C_IE(1,1)-C_IE(2,2)-C_IE(3,3)+1));
  quat(3) = 0.5*(sign(C_IE(1,3)-C_IE(3,1))*sqrt(C_IE(2,2)-C_IE(3,3)-C_IE(1,1)+1));
  quat(4) = 0.5*(sign(C_IE(2,1)-C_IE(1,2))*sqrt(C_IE(3,3)-C_IE(1,1)-C_IE(2,2)+1));
end