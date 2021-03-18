function R = get_rotation_matrix(q0,q1,q2,q3)

R = [ 1.0-2*(q2^2 + q3^2)       2*q1*q2-2*q3*q0        2*q1*q3+2*q2*q0; 
    2*q1*q2+2*q3*q0   1.0-2*(q1^2+q3^2)       2*q2*q3-2*q1*q0 ;
    2*q1*q3-2*q2*q0        2*q2*q3+2*q1*q0   1.0-2*(q1^2+q2^2) ];
