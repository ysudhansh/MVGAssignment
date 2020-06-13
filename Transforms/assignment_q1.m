function [points, res] = assignment_q1()

points = importdata("lidar.txt");
points = [points, ones(size(points,1),1)];
points = points';
K = [7.215377e+02, 0.000000e+00, 6.095593e+02; 0.000000e+00, 7.215377e+02, 1.728540e+02; 0.000000e+00, 0.000000e+00, 1.000000e+00];
X0 = [0.27; 0.06; -0.08];
R = [0. 0. 1.; -1. 0. 0.; 0. -1. 0.]';
out = K * R * [eye(3), -1 .* X0] * points;
res = [out(1,:)./out(3,:); out(2,:)./out(3,:)];
imshow("image.png");
axis on;
hold on;
colormap default;
depth = points(1,:)';
depth = (depth - min(depth)); % Did this because there are some negative values
depth = depth ./ max(depth); % Have to bring it to [0 1] range
cmap = depth;
scatter(res(1,:), res(2,:), 10, cmap, "filled");

end