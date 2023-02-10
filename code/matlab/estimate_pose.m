function P = estimate_pose(x, X)
% ESTIMATE_POSE computes the pose matrix (camera matrix) P given 2D and 3D
% points.
%   Args:
%       x: 2D points with shape [2, N]
%       X: 3D points with shape [3, N]

l = size(x, 2);
A = zeros(2 * l, 12);

% This process is similar to Homography slide 47 and computeH func form
% last assignment. On simplifying the group of equations, we get lines 22
% and 24 and then we get SVD to solve it.
count = 1;
for i = 1:l
    x1 = x(1, i);
    y1 = x(2, i);
    x_prime = X(1, i);
    y_prime = X(2, i);
    z_prime = X(3, i);

    A(count, :) = [-x_prime, -y_prime, -z_prime, -1, 0, 0, 0, 0, x1 * x_prime, x1 * y_prime, x1 * z_prime, x1];
    count = count + 1;
    A(count, :) = [0, 0, 0, 0, -x_prime, -y_prime, -z_prime, -1, y1 * x_prime, y1 * y_prime, y1 * z_prime, y1];
    count = count + 1;
end

[U, sigma, V] = svd(A);
V = V(:, end);
P = reshape(V, [4 3])';