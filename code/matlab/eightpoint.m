function F = eightpoint(pts1, pts2, M)
% eightpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.1 - Todo:
%     Implement the eightpoint algorithm
%     Generate a matrix F from correspondence '../data/some_corresp.mat'

% homogenize both pts1 and pts2

homogenised_pts1 = [];
for i = 1: length(pts1)
    point = pts1(i, :);
    homogenised_pts1(end+1, :) = [point 1];
end

homogenised_pts2 = [];
for i = 1: length(pts2)
    point = pts2(i, :);
    homogenised_pts2(end+1, :) = [point 1];
end

% Scaling with M
SCALE = [
    1 / M,  0,      0
    0,      1 / M,  0
    0,      0,      1
];

normalized_pts1 = homogenised_pts1 * SCALE';
normalized_pts2 = homogenised_pts2 * SCALE';

A = zeros(size(normalized_pts2, 1), 9);
count = 1;
for i = 1:3
    for j = 1:3
        % the x-coordinate of a point in the image is its column entry 
        % .* elementwise multiply
        A(:, count) = normalized_pts1(:, i, :) .* normalized_pts2(:, j, :);
        % A(:, count) = normalized_pts1(:, i, :)' * normalized_pts2(:, j, :);
        count = count + 1;
    end
end

% Solve for F
[U, sigma, V] = svd(A);
V = V(:, end);
F = reshape(V, [3, 3]);

% Rank 2 enforcement and recalculate F
[U, sigma, V] = svd(F);
sigma_prime = sigma;
sigma_prime(3,3) = 0;
F_prime = U * sigma_prime * V';

% refine F
refined_F = refineF(F_prime, normalized_pts1, normalized_pts2);

% final F, unnormalized
F = SCALE' * refined_F * SCALE;

