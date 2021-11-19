function log_q=quatLog(q)
% quaternion log

if norm(q) ~= 0,
    q = q/norm(q);
end
log_q = 2*acos(q(1))*q(2:4)/(norm(q(2:4))+1.e-10);