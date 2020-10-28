function q = correctionQuaternion(p)
    q0 = p(1);
    q1 = p(2);
    q2 = p(3);
    q3 = p(4);
    % theta is made according to euler's rotation theorem; it is the angle between the gravity vector and the body coordinate system
    theta = acos(2 * (q1*q3 + q0*q2)); 
    N = [ 2*(q1*q2+q0*q3) q2^2+q3^2-q0^2-q1^2 0];
    n1 = transpose(N);
    n1 = n1/norm(n1);
    %correction quaternion qc(theta,n) 
    q = transpose([cos(theta/2) n1(1,1)*sin(theta/2) n1(2,1)*sin(theta/2) n1(3,1)*sin(theta/2)]);
end