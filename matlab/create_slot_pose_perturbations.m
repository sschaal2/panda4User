function X = create_slot_pose_perturbations(fname)
% this creates an array of relative slot pose perurbations and saves this
% in a file for the sm_task

% qsfp large purtubation, coarse
max_x_mm = 12;
max_y_mm = 12;
max_z_mm = 4;
min_x_mm = -12;
min_y_mm = -12;
min_z_mm = -4;
s_x = 4;
s_y = 4;
s_z = 4;
max_orient_degree = 6;

% qsfp finer purtubation
max_x_mm = 5;
max_y_mm = 5;
max_z_mm = 4;
min_x_mm = -5;
min_y_mm = -5;
min_z_mm = -4;
s_x = 2;
s_y = 2;
s_z = 4;
max_orient_degree = 6;

% qsfp test coarse
max_x_mm = 10;
max_y_mm = 10;
max_z_mm = 3.5;
min_x_mm = -10;
min_y_mm = -10;
min_z_mm = -3.5;
s_x = 5;
s_y = 5;
s_z = 3.5;
max_orient_degree = 4;

% qsfp large purtubation, fine
max_x_mm = 12;
max_y_mm = 12;
max_z_mm = 6;
min_x_mm = -12;
min_y_mm = -12;
min_z_mm = -4;
s_x = 2;
s_y = 2;
s_z = 2;
max_orient_degree = 6;


X = [];

for x=min_x_mm:s_x:max_x_mm
    for y=min_y_mm:s_y:max_y_mm
        for z=min_z_mm:s_z:max_z_mm
            pose_x = [x y z]/1000;
            angle = randi([-max_orient_degree max_orient_degree],1)/180*pi;
            vec = rand(3,1)*2-1;
            vec = vec/norm(vec);
            pose_q = [cos(angle/2) vec'*sin(angle/2)];
            X = [X; [pose_x pose_q]];
        end
    end
end

X = [];

scalar = 2;
while length(X) < 1000;
    while 1
        x = randn/scalar * (max_x_mm - min_x_mm);
        if x<=max_x_mm && x >= min_x_mm
            break
        end
    end
    while 1
        y = randn/scalar * (max_y_mm - min_y_mm);
        if y<=max_y_mm && y >= min_y_mm
            break
        end
    end
    while 1
        z = randn/scalar * (max_z_mm - min_z_mm);
        if z<=max_z_mm && z >= min_z_mm
            break
        end
    end
    pose_x = [x y z]/1000;
    angle = randi([-max_orient_degree max_orient_degree],1)/180*pi;
    vec = rand(3,1)*2-1;
    vec = vec/norm(vec);
    pose_q = [cos(angle/2) vec'*sin(angle/2)];
    X = [X; [pose_x pose_q]];
end




save(fname,'X','-ascii');