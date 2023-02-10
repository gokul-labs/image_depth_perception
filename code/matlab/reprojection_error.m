
load('../data/someCorresp.mat');
F = eightpoint(pts1, pts2, M);
load('../data/intrinsics.mat');
E = essentialMatrix(F, K1, K2);
P1 = [K1 zeros(3,1)];
candidates_P2 = camera2(E);
prev_minimum_distance = intmax("int32");
reprojection_error_1 = 0;
reprojection_error_2 = 0;
n = size(pts1, 1);
for i = 1:4
    % from slidedeck 20-stereo
    if det(candidates_P2(1:3, 1:3, i)) ~= 1
        P2 = K2 * candidates_P2(:,:,i);
        pts3d = triangulate(P1, pts1, P2, pts2);
        points1 = P1 * pts3d';
        points2 = P2 * pts3d';
        points1 = (points1 ./ points1(3, :))';
        points2 = (points2 ./ points2(3, :))';
        % positive depth test
        if sum((pts3d(:, 3) > 0), 'all') == n
            % project the estimated 3D points back to the image 1(2) and compute the mean 
            % Euclidean error between projected 2D points and pts1(2).
            dist1 = sqrt(sum((points1(:, 1:2) - pts1) .^ 2, "all"));
            dist2 = sqrt(sum((points2(:, 1:2) - pts2) .^ 2, "all"));
            mean_dist1 = dist1 / n;
            mean_dist2 = dist2 / n;
            total_mean_dist = mean_dist1 + mean_dist2;
            if total_mean_dist < prev_minimum_distance
                final_P2 = P2;
                final_pts3d = pts3d;
                reprojection_error_1 = mean_dist1;
                reprojection_error_2 = mean_dist2;
                prev_minimum_distance = total_mean_dist;
            end
        end
    end
end

% Choose correct P2 and pts3d

P2 = final_P2;
pts3d = final_pts3d;

% estimate projections back onto respective images with final values
points1 = P1 * pts3d';
points2 = P2 * pts3d';
points1 = (points1 ./ points1(3, :))';
points2 = (points2 ./ points2(3, :))';

% Compute error
dist1 = sqrt(sum((points1(:, 1:2) - pts1) .^ 2, "all"));
dist2 = sqrt(sum((points2(:, 1:2) - pts2) .^ 2, "all"));
mean_dist1 = dist1 / n;
mean_dist2 = dist2 / n;

fprintf("Re-projection error of pts1 of someCorresp.mat %4f \n", mean_dist1);
fprintf("Re-projection error of pts2 of someCorresp.mat %4f \n", mean_dist2);

