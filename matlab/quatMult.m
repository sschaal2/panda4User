function q=quatMult(q1,q2)
% q = q2 * q1, what is q, i.e., the relative quaternion between
% both quaternions

  if norm(q1) ~= 0
    q1 = q1/norm(q1);
  end
  if norm(q2) ~= 0,
    q2 = q2/norm(q2);     
  end

  Q(1,1) =  q2(1);
  Q(1,2) = -q2(2);
  Q(1,3) = -q2(3);
  Q(1,4) = -q2(4);

  Q(2,1) =  q2(2);
  Q(2,2) =  q2(1);
  Q(2,3) = -q2(4);
  Q(2,4) =  q2(3);

  Q(3,1) =  q2(3);
  Q(3,2) =  q2(4);
  Q(3,3) =  q2(1);
  Q(3,4) = -q2(2);

  Q(4,1) =  q2(4);
  Q(4,2) = -q2(3);
  Q(4,3) =  q2(2);
  Q(4,4) =  q2(1);
  
  q = Q*q1;
  
  if q(1) < 0,
      q = -1*q;
  end