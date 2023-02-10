% A test script using templeCoords.mat
%
% Write your code here
%

% 1
im1 = imread('../data/im1.png');
im2 = imread('../data/im2.png');
load('../data/someCorresp.mat');
% 2
F = eightpoint(pts1, pts2, M);
load('../data/templeCoords.mat'); 
% 3
l = size(pts1,1);
pts2 = zeros(size(pts1));

for i = 1:l
    pts2(i, :) = epipolarCorrespondence(im1, im2, F, pts1(i, :));
end

%4
load('../data/intrinsics.mat');
E = essentialMatrix(F, K1, K2);
% P1 has to be 3x4 dimension
% 5
P1 = [K1 zeros(3,1)];

% 6
candidates_P2 = camera2(E);

% 7
prev_minimum_distance = intmax("int32");
final_pts3d = "";
final_P2 = "";
n = size(pts1, 1);

for i = 1:4
    % from slidedeck 20-stereo
    if det(candidates_P2(1:3, 1:3, i)) ~= 1
        P2 = K2 * candidates_P2(:,:,i);
        pts3d = triangulate(P1, pts1, P2, pts2);

        % estimate points on im1 and im2
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
            % combined error
            total_mean_dist = mean_dist1 + mean_dist2;
            if total_mean_dist < prev_minimum_distance
                final_P2 = P2;
                final_pts3d = pts3d;
                prev_minimum_distance = total_mean_dist;
            end
        end
    end
end

% save final correct values with least combined error
P2 = final_P2;
pts3d = final_pts3d;

% 8
x = pts3d(:, 1);
y = pts3d(:, 2);
z = pts3d(:, 3);
plot3(x,y,z, 'b*');
axis equal;

R1 = K1 \ P1(1:3, 1:3);
t1 = K1 \ P1(:, 4);
R2 = K2 \ P2(1:3, 1:3);
t2 = K2 \ P2(:, 4);

% save extrinsic parameters for dense reconstruction
save('../data/extrinsics.mat', 'R1', 't1', 'R2', 't2');
