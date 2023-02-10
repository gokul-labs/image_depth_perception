function pts3d = triangulate(P1, pts1, P2, pts2 )
% triangulate estimate the 3D positions of points from 2d correspondence
%   Args:
%       P1:     projection matrix with shape 3 x 4 for image 1
%       pts1:   coordinates of points with shape N x 2 on image 1
%       P2:     projection matrix with shape 3 x 4 for image 2
%       pts2:   coordinates of points with shape N x 2 on image 2
%
%   Returns:
%       Pts3d:  coordinates of 3D points with shape N x 3
%
l = size(pts1, 1);
pts3d = [zeros(size(pts1)), zeros(l, 2)];

for i = 1:l
    x = pts1(i, :);
    x_prime = pts2(i, :);
    x1 = x(1);
    y1 = x(2);
    x2 = x_prime(1);
    y2 = x_prime(2);
    
    % From slide 19, the solution matrix A is given by
    A = [
        x1 .* P1(3, :) - P1(1, :);
        y1 .* P1(3, :) - P1(2, :); 
        x2 .* P2(3, :) - P2(1, :);
        y2 .* P2(3, :) - P2(2, :);
    ];

    [U, sigma, V] = svd(A);
    V = V(:, end);
    V = V';

    % homogenize the result
    pts3d(i, :) = V / V(4);

end

