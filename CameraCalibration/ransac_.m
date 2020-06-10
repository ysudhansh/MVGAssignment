function ransac_()

tolerance = 0.99;
threshold = 0.5;
[points_3d, points_2d] = correspondences_2d_3d();
points_2d = points_2d';
for t = 1:size(points_3d,1)
    index_array = [];
    for j = 1:6
        index = randi(6965);
        if ismember(index, index_array)
            continue;
        else
            index_array = [index_array, index];
        end
    end
    
    p = ransac_dlt_helper(index_array);
    
    count = 0;
    inliers_index = [];
    for i=1:size(points_3d,1)
        if [double(-points_3d(i,1)), double(-points_3d(i,2)), double(-points_3d(i,3)), double(-1), 0, 0, 0, 0, double(points_2d(i,1)*points_3d(i,1)), double(points_2d(i,1)*points_3d(i,2)), double(points_2d(i,1)*points_3d(i,3)), double(points_2d(i,1)); 0, 0, 0, 0, double(-points_3d(i,1)), double(-points_3d(i,2)), double(-points_3d(i,3)), double(-1), double(points_2d(i,2)*points_3d(i,1)), double(points_2d(i,2)*points_3d(i,2)), double(points_2d(i,2)*points_3d(i,3)), double(points_2d(i,2))] * p <= [tolerance; tolerance]
            count = count + 1;
            inliers_index = [inliers_index, i];
        end
    end
    
    if count/size(points_3d,1) >= threshold
        p_est = ransac_dlt_helper(inliers_index);
        P = reshape(p_est,[4 3]);
        P = P'
        H_inf = P(:,1:3);
        h = P(:,4);
        X0 = -1 * inv(H_inf) * h
        [Q,R] = qr(inv(H_inf));
        rot = Q'
        K = inv(R);
        K = K ./ K(3,3)
        break;
    end
    
end

end
