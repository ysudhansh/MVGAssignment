function p = ransac_dlt_helper(index_array)

[points_3d, points_2d] = correspondences_2d_3d();
points_2d = points_2d';
M = double([]);
for j=1:size(index_array,2)
    M = [M; double(-points_3d(index_array(j),1)), double(-points_3d(index_array(j),2)), double(-points_3d(index_array(j),3)), double(-1), 0, 0, 0, 0, double(points_2d(index_array(j),1)*points_3d(index_array(j),1)), double(points_2d(index_array(j),1)*points_3d(index_array(j),2)), double(points_2d(index_array(j),1)*points_3d(index_array(j),3)), double(points_2d(index_array(j),1)); 0, 0, 0, 0, double(-points_3d(index_array(j),1)), double(-points_3d(index_array(j),2)), double(-points_3d(index_array(j),3)), double(-1), double(points_2d(index_array(j),2)*points_3d(index_array(j),1)), double(points_2d(index_array(j),2)*points_3d(index_array(j),2)), double(points_2d(index_array(j),2)*points_3d(index_array(j),3)), double(points_2d(index_array(j),2))];
end

% [U,D,V] = svd(M);
[V,D] = eig(M'*M);
p = V(:,1); % Since we need the eigenvector corresponding to the smallest eigenvalue, similar to how we need the singular vector corresponding to the smallest singular value (singular value = (eigenvalue)^2)

end