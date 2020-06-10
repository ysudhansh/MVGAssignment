function dlt()

[points_3d, points_2d] = correspondences_2d_3d();
points_2d = points_2d';
M = double([]);
for i=1:size(points_3d,1)
    M = [M; double(-points_3d(i,1)), double(-points_3d(i,2)), double(-points_3d(i,3)), double(-1), 0, 0, 0, 0, double(points_2d(i,1)*points_3d(i,1)), double(points_2d(i,1)*points_3d(i,2)), double(points_2d(i,1)*points_3d(i,3)), double(points_2d(i,1)); 0, 0, 0, 0, double(-points_3d(i,1)), double(-points_3d(i,2)), double(-points_3d(i,3)), double(-1), double(points_2d(i,2)*points_3d(i,1)), double(points_2d(i,2)*points_3d(i,2)), double(points_2d(i,2)*points_3d(i,3)), double(points_2d(i,2))];
end

% [U,D,V] = svd(M);
[V,D] = eig(M'*M);
p = V(:,1); % Since we need the eigenvector corresponding to the smallest eigenvalue, similar to how we need the singular vector corresponding to the smallest singular value (singular value = (eigenvalue)^2)
P = reshape(p,[4 3]);
P = P'
H_inf = P(:,1:3);
h = P(:,4);
X0 = -1 * inv(H_inf) * h;
[Q,R] = qr(inv(H_inf));
rot = Q'
K = inv(R);
K = K ./ K(3,3)

end