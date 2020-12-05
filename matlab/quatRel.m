function q=quatRel(qf,qs)
% if qf = q * qs, what is q, i.e., the relative quaternion between
% both quaternions

  qs(2:4) = -qs(2:4);
  
  q = quatMult(qs,qf);