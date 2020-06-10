function [points_3d, points_2d] = correspondences_2d_3d()

points_3d = double(importdata("../Transforms/lidar.txt"));
points = [points_3d, double(ones(size(points_3d,1),1))];
points = points';
K = [7.215377e+02, 0.000000e+00, 6.095593e+02; 0.000000e+00, 7.215377e+02, 1.728540e+02; 0.000000e+00, 0.000000e+00, 1.000000e+00];
X0 = double([0.27; 0.06; -0.08]);
R = double([0. 0. 1.; -1. 0. 0.; 0. -1. 0.])';
out = K * R * [eye(3), -1 .* X0] * points;
points_2d = [out(1,:)./out(3,:); out(2,:)./out(3,:)];

end