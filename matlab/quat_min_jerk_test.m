function quat_min_jerk_test

q_start = [0 1 0 0]';
q_end   = [0 0 1 0]';
q_start = randn(4,1);
q_start = q_start*sign(q_start(1));
q_end = randn(4,1);
q_end = q_end*sign(q_end(1));
t = q_end'*q_start;
if (t < 0)
    q_end = -q_end;
end
q_start = q_start/norm(q_start);
q_end = q_end/norm(q_end);


theta = acos(q_start'*q_end);
Q=[];
Q1=[];
QQ=[];
W=[];
WW=[];
S=[];
s=0;
sd=0;
sdd=0;
q_last = q_start;

q = q_start;
qd = q*0;
qdd = q*0;
qt = q_end;
qtd = qt*0;
qtdd = qt*0;

for i=1:101
    u = (i-1)/101;
    [s,sd,sdd] = min_jerk_next_step(s,sd,sdd,1,0,0,1-u,0.01);
    [q,qd,qdd] = min_jerk_quat_next_step(q,qd,qdd,qt,qtd,qtdd,1-u,0.01);
    S(i,:)=[s sd sdd],
    q_slurp=(q_start*sin((1-s)*theta)/(sin(theta)+1e-10) + q_end*sin(s*theta)/(sin(theta)+1.e-10));
    q_slurp = q_slurp*sign(q_slurp(1));
    Q(i,:)=q_slurp';
    %Q1(i,:)=(q_start*sin((1-u)*theta)/(sin(theta)+1e-10) + q_end*sin(u*theta)/(sin(theta)+1.e-10))';
    W(i,:)=quatLog(quatRel(Q(i,:)',q_last))/0.01;
    [ad,add]=angularFromQuat(q,qd,qdd);
    WW(i,:) = ad';
    q_last = Q(i,:)';
    % q_start = Q(i,:)'
    % theta = acos(q_start'*q_end);
    QQ(i,:) = q';
end

figure(1)
clf;
plot(Q);
hold on;
%plot(Q1,'-.');
plot(QQ,'-.','LineWidth',2);
hold off;
figure(2),
plot(sum(Q.*Q,2));
figure(3);
plot(S);
figure(4);
plot(W);
hold on;
plot(WW,'-.','LineWidth',2);
hold off;



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

  tau  = t_togo;
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
  

function [q_next,qd_next, qdd_next] = min_jerk_quat_next_step (q,qd,qdd,qt,qtd,qtdd,t_togo,dt)

  % a safety check
  if (dt > t_togo || dt <= 0) 
      q_next = qt,
      qd_next = qtd;
      qdd_next = qtdd;
    return
  end

  t1 = dt;
  t2 = t1 * dt;
  t3 = t2 * dt;
  t4 = t3 * dt;
  t5 = t4 * dt;

  tau  = t_togo;edit
  tau1 = t_togo;
  tau2 = tau1 * tau;
  tau3 = tau2 * tau;
  tau4 = tau3 * tau;
  tau5 = tau4 * tau;

  % calculate the constants
  dist       = quatRel(qt,q);
  dist       = quatLog(dist);
  [ad,add]   = angularFromQuat(q,qd,qdd);
  [atd,atdd] = angularFromQuat(qt,qtd,qtdd);

  a1t2   = atdd;
  a0t2   = add;
  v1t1   = atd;
  v0t1   = ad;
  
  c1 = 6.*dist/tau5 + (a1t2 - a0t2)/(2.*tau3) - 3.*(v0t1 + v1t1)/tau4;
  c2 = -15.*dist/tau4 + (3.*a0t2 - 2.*a1t2)/(2.*tau2) + (8.*v0t1 + 7.*v1t1)/tau3; 
  c3 = 10.*dist/tau3+ (a1t2 - 3.*a0t2)/(2.*tau) - (6.*v0t1 + 4.*v1t1)/tau2; 
  c4 = add/2.;
  c5 = ad;
%  c6 = x; ??
  
  delta_a_next   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1; % + c6;
  aux = norm(delta_a_next)/2;
  q_rel = [cos(aux); delta_a_next/2*sin(aux)/aux];
  q_next = quatMult(q,q_rel);
  q_next = q_next*sign(q_next(1));
  q_next = q_next/norm(q_next);
  ad_next  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
  add_next = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;
  [qd_next,qdd_next] = quatFromAngular(q_next,ad_next,add_next);
  
  
  
