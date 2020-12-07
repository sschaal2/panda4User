function R=quatToRotMat(q)
% converts quaterenion to rotation matrix

Q0=1;
Q1=2;
Q2=3;
Q3=4;

  R(1,1) = -1.0 + 2.0*q(Q0)^2 + 2.0*q(Q1)^2;
  R(2,2) = -1.0 + 2.0*q(Q0)^2 + 2.0*q(Q2)^2;
  R(3,3) = -1.0 + 2.0*q(Q0)^2 + 2.0*q(Q3)^2;

  R(1,2) = 2.0 * (q(Q1)*q(Q2) + q(Q0)*q(Q3));
  R(1,3) = 2.0 * (q(Q1)*q(Q3) - q(Q0)*q(Q2));
  R(2,1) = 2.0 * (q(Q1)*q(Q2) - q(Q0)*q(Q3));
  R(2,3) = 2.0 * (q(Q2)*q(Q3) + q(Q0)*q(Q1));
  R(3,1) = 2.0 * (q(Q1)*q(Q3) + q(Q0)*q(Q2));
  R(3,2) = 2.0 * (q(Q2)*q(Q3) - q(Q0)*q(Q1));
