function vl_stats(fname)

% quick analysis and plots of vl correction results

% load the statistics matrix and extract relevant variabls
M = load (fname,'-ascii');

c = 0;
base_ref_x = M(:,c*7+1:c*7+3);
base_ref_q = M(:,c*7+4:c*7+7);

c=c+1;
ref_x = M(:,c*7+1:c*7+3);
ref_q = M(:,c*7+4:c*7+7);

c=c+1;
adj_delta_x = M(:,c*7+1:c*7+3);
adj_delta_q = M(:,c*7+4:c*7+7);

c=c+1;
target_x = M(:,c*7+1:c*7+3);
target_q = M(:,c*7+4:c*7+7);

c=c+1;
current_x = M(:,c*7+1:c*7+3);
current_q = M(:,c*7+4:c*7+7);

c=c+1;
vl_delta_x = M(:,c*7+1:c*7+3);
vl_delta_q = M(:,c*7+4:c*7+7);

c=c+1;
corrected_target_x = M(:,c*7+1:c*7+3);
corrected_target_q = M(:,c*7+4:c*7+7);

c=c+1;
corrected_current_x = M(:,c*7+1:c*7+3);
corrected_current_q = M(:,c*7+4:c*7+7);

% computer the error relative of the correction given our knowledge of the
% perfect delte_pose perturbation

err_x = zeros(size(base_ref_x));
err_q = zeros(size(base_ref_q));
err_log_q = zeros(size(base_ref_x));

for i=1:length(err_x)
    % remaining error after vl correction
    err_x(i,:) = base_ref_x(i,:) - (ref_x(i,:)+vl_delta_x(i,:));
    % recover the delta pose perturbation: must be identical to pertubation
    % table
    delta_x(i,:) = ref_x(i,:) - base_ref_x(i,:);
    % recover the perfect target before insertation after tilted move:
    % this must be expressed in the ref system:
    R=quatToRotMat(ref_q(i,:)');
    temp = R*(target_x(i,:)-ref_x(i,:))';
    R=quatToRotMat(base_ref_q(i,:)');
    opt_target_x(i,:) = (R'*temp)'+base_ref_x(i,:);
end

for i=1:length(err_q)
    % the vl correction applied to perturbed reference posse
    q_adj = quatMult(ref_q(i,:)',vl_delta_q(i,:)');
    % recover the delta pose pertubation: must be identical to purturbation
    % table
    delta_q(i,:) = quatRel(ref_q(i,:)',base_ref_q(i,:)')';
    log_delta_q(i,:) = quatLog(delta_q(i,:)')';
    % the remaining orientation error after vl correction
    err_q(i,:) = quatRel(base_ref_q(i,:)',q_adj)';
    err_log_q(i,:) = quatLog(err_q(i,:)')';
    % recover the perfect target before insertation after tilted move: this
    % must be expressed in ref system:tbd
    R=quatToRotMat(ref_q(i,:)');
    temp = quatRel(target_q(i,:)',ref_q(i,:)');
    temp(2:4) = R*temp(2:4);
    opt_target_q(i,:) = temp';
    R=quatToRotMat(base_ref_q(i,:)');
    temp(2:4) = R'*temp(2:4);
    opt_target_q(i,:) = quatMult(base_ref_q(i,:)',temp)';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% first figure is about delta pose correction errors
fig1=figure(1);
clf;
sgtitle({fname,'DL Correction Error'},'Interpreter','none');

% position error relative to known perfect pose
subplot(221);
boxplot(err_x,'labels',{'p_err_x','p_err_y','p_err_z'});
ylabel('error[m]')
title({'Position Error','after correction'});

% 3D plot of position error
subplot(222);
plot3D(ref_x-base_ref_x,'.');
hold on;
plot3D(err_x,'.');
title({'Position Error before/after','correction [with 4mm cube]'});
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');

% draw cube
center = -[1 1 1]*0.002;
cubesize = 0.004;
X = [0 0 1 1 0 0 1 1 1 1 1 1 0 0 0 0 0]';
Y = [0 1 1 0 0 0 0 0 0 1 1 1 1 1 1 0 0]';
Z = [0 0 0 0 0 1 1 0 1 1 0 1 1 0 1 1 0]';
X1 = [X*cubesize+center(1)];
Y1 = [Y*cubesize+center(2)];
Z1 = [Z*cubesize+center(3)];
plot3(X1,Y1,Z1);

axis('equal','tight');
grid on;
hold off;

% orientation error relative to known perfect pose
subplot(223);
boxplot(err_log_q/pi*180,'labels',{'o_err_x','o_err_y','o_err_z'});
title({'Qrientation Error','after correction'});
ylabel('error[deg]');

% 3D plot of orientation errors
subplot(224);

plot3D(log_delta_q/pi*180,'.');
hold on;
plot3D(err_log_q/pi*180,'.');

% draw cube
center = -[1 1 1]*1;
cubesize = 2;
X = [0 0 1 1 0 0 1 1 1 1 1 1 0 0 0 0 0]';
Y = [0 1 1 0 0 0 0 0 0 1 1 1 1 1 1 0 0]';
Z = [0 0 0 0 0 1 1 0 1 1 0 1 1 0 1 1 0]';
X1 = [X*cubesize+center(1)];
Y1 = [Y*cubesize+center(2)];
Z1 = [Z*cubesize+center(3)];
plot3(X1,Y1,Z1);

xlabel('x[deg]');
ylabel('y[deg]');
zlabel('z[deg]');
axis('equal','tight');
grid on;
hold off;

title({'Orientation error before/after','correction [with 2 degree cube]'});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% the second figure is about pos tracking errors
fig2 = figure(2);
clf;
sgtitle({fname,'Tracking Position Error Before Insertion'},'Interpreter','none');

% boxplots of errors
subplot(221);
pos_track_err = current_x - target_x;
boxplot(pos_track_err,'labels',{'p_err_x','p_err_y','p_err_z'});
title({'Tracking Error','before correction'});
ylabel('error[m]');

% 3D plot of position error
subplot(222);
plot3D(pos_track_err,'.');
title({'Tracking Error','before correction'});
grid on;
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
hold on;
plot3(0,0,0,'ro');
hold off;
axis('equal');

subplot(223);
corrected_pos_track_err = corrected_current_x - corrected_target_x;
boxplot(corrected_pos_track_err,'labels',{'p_err_x','p_err_y','p_err_z'});
title({'Tracking Error','after correction'});
ylabel('error[m]');

% 3D plot of position error
subplot(224);
plot3D(corrected_pos_track_err,'.');
title({'Tracking Error','after correction'});
grid on;
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
hold on;
plot3(0,0,0,'ro');
hold off;
axis('equal');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% the third figure is about orient tracking errors
fig3 = figure(3);
clf;
sgtitle({fname,'Tracking Orientation Error Before Insertion'},'Interpreter','none');

% boxplots of errors
subplot(221);
for i=1:length(current_q),
    orient_track_err(i,:) = quatLog(quatRel(current_q(i,:)',target_q(i,:)'))';
end
boxplot(orient_track_err/pi*180,'labels',{'o_err_wx','o_err_wy','o_err_wz'});
title({'Orientation Tracking Error','before correction'});
ylabel('error[deg]');

% 3D plot of position error
subplot(222);
plot3D(orient_track_err/pi*180,'.');
title({'Orientation Tracking Error','before correction'});
grid on;
axis('equal');
xlabel('x[deg]');
ylabel('y[deg]');
zlabel('z[deg]');
hold on;
plot3(0,0,0,'ro');
hold off;

subplot(223);
for i=1:length(corrected_current_q),
    corrected_orient_track_err(i,:) = quatLog(quatRel(corrected_current_q(i,:)',corrected_target_q(i,:)'))';
end
boxplot(corrected_orient_track_err/pi*180,'labels',{'o_err_wx','o_err_wy','o_err_wz'});
title({'Orientation Tracking Error','after correction'});
ylabel('error[deg]');

% 3D plot of position error
subplot(224);
plot3D(corrected_orient_track_err/pi*180,'.');
title({'Orientation Tracking Error','after correction'});
axis('equal');
grid on;
xlabel('x[rad]');
ylabel('y[rad]');
zlabel('z[rad]');
hold on;
plot3(0,0,0,'ro');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure 4 is errors from total error from corrections in
% residual plots
fig4=figure(4);
clf;
sgtitle({fname,'DL Total Errors'},'Interpreter','none');

% plot total error agains data ID
subplot(311);
total_error_x = sqrt(sum(err_x.^2,2));
bar(total_error_x);
ylabel('total error[m]')
title({'Total Error after correction'});
xlabel('Data ID Number');

% plot total orientation error against data ID
subplot(312);
total_error_log_q = sqrt(sum((err_log_q/pi*180).^2,2));
bar(total_error_log_q);
title({'Total Qrientation Error after correction'});
ylabel('total error[deg]');
xlabel('Data ID Number');

% residual plot in positions
subplot(325);
plot(delta_x,total_error_x,'*');
legend('x','y','z','Location','southeast');
xlabel('delta position pertubation [m]');
ylabel('total error[m]');
title('Residual Plots Position');

% residual plot in orientations
subplot(326);
plot(log_delta_q*180/pi,total_error_log_q,'*');
legend('wx','wy','wz','Location','southeast');
xlabel('delta orientation pertubation [rad]');
ylabel('total error[rad]');
title('Residual Plots Orientation');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% the fith figure is about the error relative to the optimal pose
% before inseration (pose without socket perturbation)
fig5 = figure(5);
clf;
sgtitle({fname,'Position/Orientation Error Relative to Optimal Pose'},'Interpreter','none');

% boxplots of errors
subplot(221);
pos_track_err = corrected_current_x - opt_target_x;
boxplot(pos_track_err,'labels',{'p_err_x','p_err_y','p_err_z'});
title({'Position Error to Optimal','after correction'});
ylabel('error[m]');

% 3D plot of position error
subplot(222);
plot3D(pos_track_err,'.');
title({'Position Error to Optimal','after correction'});
grid on;
axis('equal');
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
hold on;
plot3(0,0,0,'ro');
hold off;

subplot(223);
for i=1:length(corrected_current_q),
    corrected_orient_track_err(i,:) = quatLog(quatRel(corrected_current_q(i,:)',opt_target_q(i,:)'))';
end
boxplot(corrected_orient_track_err/pi*180,'labels',{'o_err_wx','o_err_wy','o_err_wz'});
title({'Orientation Error to Optimal','after correction'});
ylabel('error[deg]');

% 3D plot of position error
subplot(224);
plot3D(corrected_orient_track_err/pi*180,'.');
title({'Orientation Error to Optimal','after correction'});
axis('equal');
grid on;
xlabel('x[rad]');
ylabel('y[rad]');
zlabel('z[rad]');
hold on;
plot3(0,0,0,'ro');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% create nice output pdf file
outfname = [fname,'.pdf'];
delete(outfname);
print(fig1,'-dpdf','-fillpage','/tmp/t1');
print(fig2,'-dpdf','-fillpage','/tmp/t2');
print(fig3,'-dpdf','-fillpage','/tmp/t3');
print(fig4,'-dpdf','-fillpage','/tmp/t4');
print(fig5,'-dpdf','-fillpage','/tmp/t5');
append_pdfs(outfname,'/tmp/t1.pdf','/tmp/t4.pdf','/tmp/t2.pdf','/tmp/t3.pdf','/tmp/t5.pdf');


