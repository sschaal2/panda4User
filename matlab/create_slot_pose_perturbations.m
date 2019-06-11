function X = create_slot_pose_perturbations(fname)
% this creates an array of relative slot pose perurbations and saves this
% in a file for the sm_task

max_x_mm = 2/2;
max_y_mm = 2/2;
max_z_mm = 4/2;
max_orient_degree = 5/2.5;

max_x_mm = 10;
max_y_mm = 10;
max_z_mm = 5;
max_orient_degree = 5;

X = [];

s = 2;

for x=-max_x_mm:s:max_x_mm
    for y=-max_y_mm:s:max_y_mm
        for z=-max_z_mm:s:max_z_mm
            pose_x = [x y z]/1000;
            angle = randi([-max_orient_degree max_orient_degree],1)/180*pi;
            vec = rand(3,1)*2-1;
            vec = vec/norm(vec);
            pose_q = [cos(angle/2) vec'*sin(angle/2)];
            X = [X; [pose_x pose_q]];
        end
    end
end

save(fname,'X','-ascii');