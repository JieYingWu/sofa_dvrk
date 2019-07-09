%% Base
fixed = make_transform([-0.25 0 0.5], [0 0 pi]);
l0 = make_transform([0.039 -0.40788 -0.07879], [pi/2 0 pi]);
base_pos=fixed*l0;
base_pos(1:3,4)'
rotm2quat(base_pos(1:3,1:3))

%% Link 1
fk1 = make_transform([0 0 0], [-pi/2 -pi/2, pi]);
j1 = make_transform([0 0 0], [0 -pi/2 pi/2]);
l1 = make_transform([0.0125 0 0.526], [pi 0 pi/2]);
l1_pos = base_pos * j1 * l1;
l1_pos(1:3,4)'
rotm2quat(l1_pos(1:3,1:3))