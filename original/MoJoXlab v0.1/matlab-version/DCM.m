function M = DCM(q)
%convert from unit quaternions to direction cosine matrix
q = q/norm(q);
M = [q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2    2 * (q(2)*q(3) - q(1)*q(4))         2 * (q(2)*q(4) + q(1)*q(3));
     2 * (q(2)*q(3) + q(1)*q(4))          q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2   2 * (q(3)*q(4) - q(1)*q(2));
     2 * (q(2)*q(4) - q(1)*q(3))          2 * (q(3)*q(4) + q(1)*q(2))         q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
end