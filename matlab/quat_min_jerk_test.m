function quat_min_jerk_test

q_start = [0 1 0 0]';
q_end   = [0 0 1 0]';
q_start = randn(4,1);
q_end = randn(4,1);
q_start = q_start/norm(q_start);
q_end = q_end/norm(q_end);


theta = acos(q_start'*q_end);
Q=[];
Q1=[];
s=0;
sd=0;
sdd=0;

for i=1:101
    u = (i-1)/101;
    [s,sd,sdd] = min_jerk_next_step(s,sd,sdd,1,0,0,1-u,0.01);
    Q(i,:)=(q_start*sin((1-s)*theta)/(sin(theta)+1e-10) + q_end*sin(s*theta)/(sin(theta)+1.e-10))';
    Q1(i,:)=(q_start*sin((1-u)*theta)/(sin(theta)+1e-10) + q_end*sin(u*theta)/(sin(theta)+1.e-10))';
    q_start = Q(i,:)'
    theta = acos(q_start'*q_end);
end

figure(1)
plot(Q);
hold;
plot(Q1);
hold;
figure(2),
plot(sum(Q.*Q,2));



function [x_next,xd_next, xdd_next] = min_jerk_next_step (x,xd,xdd,t,td,tdd,t_togo,dt)

  % a safety check
  if (dt > t_togo || dt <= 0) 
      x_next = t,
      xd_next = td;
      xdd_next = tdd;
    return
  end

  t1 = dt;
  t2 = t1 * dt;
  t3 = t2 * dt;
  t4 = t3 * dt;
  t5 = t4 * dt;

  tau = t_togo;
  tau1 = t_togo;
  tau2 = tau1 * tau;
  tau3 = tau2 * tau;
  tau4 = tau3 * tau;
  tau5 = tau4 * tau;

  % calculate the constants
  dist   = t - x;
  p1     = t;
  p0     = x;
  a1t2   = tdd;
  a0t2   = xdd;
  v1t1   = td;
  v0t1   = xd;
  
  c1 = 6.*dist/tau5 + (a1t2 - a0t2)/(2.*tau3) - 3.*(v0t1 + v1t1)/tau4;
  c2 = -15.*dist/tau4 + (3.*a0t2 - 2.*a1t2)/(2.*tau2) + (8.*v0t1 + 7.*v1t1)/tau3; 
  c3 = 10.*dist/tau3+ (a1t2 - 3.*a0t2)/(2.*tau) - (6.*v0t1 + 4.*v1t1)/tau2; 
  c4 = xdd/2.;
  c5 = xd;
  c6 = x;
  
  x_next   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1 + c6;
  xd_next  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
  xdd_next = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;
  
