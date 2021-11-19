function [qd,qdd]=quatFromAngular(q,ad,add)
% qd = 0.5 * Q(q) ad, with the definition of the matrix Q below
% qdd = 0.5 Q(q) add + 0.5 Q(qd) ad

if norm(q) ~= 0
    q = q/norm(q);
end

qd = 0.5*createQMatrix(q)*ad;
qdd = 0.5*createQMatrix(q)*add + 0.5*createQMatrix(qd)*ad;

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
