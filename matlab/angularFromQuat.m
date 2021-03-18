function [ad,add]=angularFromQuat(q,qd,qdd)
% ad = 2 Q(q)' qd, with the definition of the matrix Q below
% add = Q(q)' (2 qdd - Q(qd) ad)

if norm(q) ~= 0
    q = q/norm(q);
end

ad = 2*createQMatrix(q)'*qd;
add = createQMatrix(q)'*(2*qdd - createQMatrix(qd)*ad);

% special Q matrix for quaternion calculus
function Q = createQMatrix(vec4)
Q(1,1) = -vec4(2);
Q(1,2) = -vec4(3);
Q(1,3) = -vec4(4);

Q(2,1) =  vec4(1);
Q(2,2) =  vec4(4);
Q(2,3) = -vec4(3);

Q(3,1) = -vec4(4);
Q(3,2) =  vec4(1);
Q(3,3) =  vec4(2);

Q(4,1) =  vec4(3);
Q(4,2) = -vec4(2);
Q(4,3) =  vec4(1);
