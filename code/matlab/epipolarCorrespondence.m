function [pts2] = epipolarCorrespondence(im1, im2, F, pts1)
% epipolarCorrespondence:
%   Args:
%       im1:    Image 1
%       im2:    Image 2
%       F:      Fundamental Matrix from im1 to im2
%       pts1:   coordinates of points in image 1
%   Returns:
%       pts2:   coordinates of points in image 2
%

% homogenize points

homogenised_pts1 = [];
for i = 1: size(pts1, 1)
    point = pts1(i, :);
    homogenised_pts1(end+1, :) = [point 1];
end

% from slides l' = F * x. Change dimension to allow matrix multiplication
homogenised_pts1 = homogenised_pts1';
line = F * homogenised_pts1;

% choose a point
point = homogenised_pts1(:, 1);
point = round(point);

% select a small window of size w around the point x ==> w = 5 here
y_range_1 = (point(2) - 5 : point(2) + 5);
x_range_1 = (point(1) - 5 : point(1) + 5);

candidate_points_1 = im1(y_range_1, x_range_1, :);

% Handle border ranges
search_range_start = point(1) - 5;
if search_range_start < 0
    search_range_start = 0;
end
search_range_end = point(1) + 5;
if search_range_end > size(im1, 2)
    search_range_end = size(im1, 2);
end

% set default min distance to measure closest points
prev_minimum_distance = intmax("int32");

% ax + by + c = 0
% y = -(ax+c) / b
% [homogenised_pts1(1), - ((line(1) * homogenised_pts1(1)) + line(3)) / line(2)];

for x = search_range_start:search_range_end
    
    point = round([x, - ((line(1) * x) + line(3)) / line(2), 1]);
    a = (point(2)-5);
    if a < 0
        a = 0;
    end
    b = (point(1)-5);
    if b < 0
        b = 0;
    end
    y_range_2 = (a : point(2) + 5);
    x_range_2 = (b : point(1) + 5);

    if a > 0 && b > 0
        candidate_points_2 = im2(y_range_2, x_range_2, :);
    
        curr_euclidean_dist = sqrt(sum((candidate_points_1 - candidate_points_2) .^ 2, "all"));
        if curr_euclidean_dist < prev_minimum_distance
            prev_minimum_distance = curr_euclidean_dist;
            pts2 = [point(1), point(2)];
        end
    end
end
